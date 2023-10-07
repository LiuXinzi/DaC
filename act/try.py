import torch
import numpy as np
import os
import pickle
import argparse
import matplotlib.pyplot as plt
from copy import deepcopy
from tqdm import tqdm
from einops import rearrange

from constants import DT
from constants import PUPPET_GRIPPER_JOINT_OPEN
from utils import load_data # data functions
from utils import sample_box_pose, sample_insertion_pose # robot functions
from utils import compute_dict_mean, set_seed, detach_dict # helper functions
from policy import ACTPolicy, CNNMLPPolicy
from visualize_episodes import save_videos
import time
import pyrealsense2 as rs
import cv2
import numpy as np
from pymycobot import ElephantRobot
import sys
import re
from PIL import Image
import os
import numpy as np
import h5py
import os
from sim_env import BOX_POSE
import IPython
e = IPython.embed


    


def make_policy(policy_class, policy_config):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    elif policy_class == 'CNNMLP':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer


def main(args):
    ckpt_name='policy_best.ckpt'
    ckpt_dir='/home/dc/python/act'
    policy_class='ACT'
    enc_layers = 4
    dec_layers = 7
    nheads = 8
    state_dim=6
    tate_dim = 14
    lr_backbone = 1e-5
    backbone = 'resnet18'
    policy_config = {'lr': 1e-5,
                         'num_queries': 100,
                         'kl_weight': 10,
                         'hidden_dim': 512,
                         'dim_feedforward': 3200,
                         'lr_backbone': lr_backbone,
                         'backbone': backbone,
                         'enc_layers': enc_layers,
                         'dec_layers': dec_layers,
                         'nheads': nheads,
                         'camera_names': ['top'],
                         }

    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    policy = make_policy(policy_class, policy_config)
    loading_status = policy.load_state_dict(torch.load(ckpt_path))
    policy.cuda()
    policy.eval()
    # connect with camera and robot 
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)

    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    post_process = lambda a: a * stats['action_std'] + stats['action_mean']
    
 
    # set the robot and others
    pipeline = rs.pipeline()
    config = rs.config()
    # 配置:  代码已经以30Hz的频率采集图像
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)  
    pipeline.start(config)
    follower = ElephantRobot("10.24.179.109", 5001)
    follower.start_client()
    max_timesteps=10000
    query_frequency=100
    all_time_actions = torch.zeros([max_timesteps, max_timesteps+query_frequency, state_dim]).cuda()
    with torch.inference_mode():
        for t in range(max_timesteps):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
             continue
            # 将RealSense摄像头捕获的图像数据转换为OpenCV格式
            color_image = np.asanyarray(color_frame.get_data())
            color_image = color_image.transpose((2, 0, 1))
            color_image = torch.from_numpy(color_image/255.0).float().cuda().unsqueeze(0).unsqueeze(0)
             
            qpos_raw=np.array(follower.get_angles())
            qpos = pre_process(qpos_raw)
            qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
            
            if t % query_frequency == 0:
                all_actions = policy(qpos, color_image)
            all_time_actions[[t], t:t+query_frequency] = all_actions
            actions_for_curr_step = all_time_actions[:, t]
            actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
            actions_for_curr_step = actions_for_curr_step[actions_populated]
            k = 0.01
            exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
            exp_weights = exp_weights / exp_weights.sum()
            exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
            raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
            raw_action = raw_action.squeeze(0).cpu().numpy()
            action = post_process(raw_action)
            print(action)
            follower.write_angles(action,1900)
            
            # raw_action = all_actions[:, t % query_frequency]
            # raw_action = raw_action.squeeze(0).cpu().numpy()
            # #action = post_process(raw_action)
            # target_qpos = raw_action+np.array(qpos_raw)
            # print(target_qpos)
            # follower.write_angles(target_qpos,1900)
            # follower.command_wait_done()
            # t+=1

    



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--onscreen_render', action='store_true')
    parser.add_argument('--ckpt_dir', action='store', type=str, help='ckpt_dir', required=True)
    parser.add_argument('--policy_class', action='store', type=str, help='policy_class, capitalize', required=True)
    parser.add_argument('--task_name', action='store', type=str, help='task_name', required=True)
    parser.add_argument('--batch_size', action='store', type=int, help='batch_size', required=True)
    parser.add_argument('--seed', action='store', type=int, help='seed', required=True)
    parser.add_argument('--num_epochs', action='store', type=int, help='num_epochs', required=True)
    parser.add_argument('--lr', action='store', type=float, help='lr', required=True)

    # for ACT
    parser.add_argument('--kl_weight', action='store', type=int, help='KL Weight', required=False)
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size', required=False)
    parser.add_argument('--hidden_dim', action='store', type=int, help='hidden_dim', required=False)
    parser.add_argument('--dim_feedforward', action='store', type=int, help='dim_feedforward', required=False)
    parser.add_argument('--temporal_agg', action='store_true')
    
    main(vars(parser.parse_args()))
