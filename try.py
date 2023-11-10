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
# import module
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
import pyrealsense2 as rs
import cv2
import numpy as np
import time
import os
import sys
from pymycobot import ElephantRobot
import os
import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
from dynamixel_sdk import * # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_GOAL_CURRENT          = 102
    ADDR_GOAL_VELOCITY         = 104
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_CURRENT       = 126
    ADDR_PRESENT_VELOCITY      = 128
    DXL_MINIMUM_POSITION_VALUE  =3367         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 2330    # Refer to the Maximum Position Limit of product eManual
    DXL_MINIMUM_CURRENT_VALUE  = 100
    DXL_MAXIMUM_CURRENT_VALUE  = 100
    BAUDRATE                    = 57600

ISOPEN=False
# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
dxl_goal_current = [DXL_MINIMUM_CURRENT_VALUE, DXL_MAXIMUM_CURRENT_VALUE] 

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Set Current-based Position Control mode
# Modifying EEPROM area value should be done before enabling DYNAMIXEL torque.
# Address of Operating Mode : 11
# Current-based Position Control mode value : 5
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 11, 5)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("DYNAMIXEL has been successfully configured as Current-based Position Control Mode.")

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

def close_port():
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS: 
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    # Close port
    portHandler.closePort()
    print('Closed')
def open_g():
    global ISOPEN
    if ISOPEN:
        print('pass')
    else:
        print('start open')
        ISOPEN=True
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current[0])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[0])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        while 1:
            # Read present position
            
            
            dxl_present_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            dxl_present_vel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))


            # cnt += 1
            if not abs(dxl_goal_position[0] - dxl_present_pos) > DXL_MOVING_STATUS_THRESHOLD:
                break
            if abs(dxl_present_vel) < 10 :
                break
def close_g():
    global ISOPEN
    if not ISOPEN:
        print('pass')
    else :
        ISOPEN=False
        print('start close')
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current[1])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[1])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        while 1:
            # Read present position
            
            
            dxl_present_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            dxl_present_vel, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))


            # cnt += 1
            if not abs(dxl_goal_position[1] - dxl_present_pos) > DXL_MOVING_STATUS_THRESHOLD:
              
                break
            if abs(dxl_present_vel) < 10 :
               
                break

open_g()
getch()
class Realsense2:
    def __init__(self, camera_id_list = [0], camera_width=1280, camera_height=720, camera_fps=30):
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps
        self.camera_id_list = camera_id_list
        

    def camera_config(self):
        self.connect_device = []
        # get all realsense serial number
        for d in rs.context().devices:
            
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                self.connect_device.append(d.get_info(rs.camera_info.serial_number))
        # config realsense
        self.pipeline_list = [rs.pipeline() for i in range(len(self.camera_id_list))]
        self.config_list = [rs.config() for i in range(len(self.camera_id_list))]
        if len(self.camera_id_list) == 1: # one realsense
                self.config_list[0].enable_device(self.connect_device[0])
                self.config_list[0].enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, self.camera_fps)
                self.config_list[0].enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, self.camera_fps)
                self.pipeline_list[0].start(self.config_list[0])
        elif len(self.camera_id_list) >= 2: # two realsense
            if len(self.connect_device) < 2:
                print('Registrition needs two camera connected.But got one.Please run realsense-viewer to check your camera status.')
                exit()
            # enable config
            for n, config in enumerate(self.config_list):
                config.enable_device(self.connect_device[n])
                config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, self.camera_fps)
                config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, self.camera_fps)
                # self.config_list[n] = config
            # start pipeline
            for n, pipeline in enumerate(self.pipeline_list):
                pipeline.start(self.config_list[n])

    def wait_frames(self, frame_id=None):
        '''
        camera_id:
            @ = camera number , get all frame
            @ = id , get specific id camera frame 
        '''
        self.frame_list = [None for i in range(len(self.camera_id_list))]
        if frame_id != None: # give a frame id
            self.frame_list[n] = self.pipeline_list[frame_id].wait_for_frames()
        else: # get all frame
            if len(self.camera_id_list) == 1:
                self.frame_list.append(self.pipeline_list[0].wait_for_frames())
            else:
                for n, camera_id in enumerate(self.camera_id_list):
                    self.frame_list[n] = self.pipeline_list[n].wait_for_frames()

    def rgb_image(self, camera_id=0):
        color_frame = self.frame_list[camera_id].get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def depth_frame(self, frames):
        depth_frame = frames.get_depth_frame()
        return depth_frame

    def stop(self):
        for pipeline in self.pipeline_list:
            pipeline.stop()

def last_process(input_array):
    # 取前六个数字作为新数组
    first_six = input_array[:-1]
    
    
    # 判断最后一个数字是否大于0.5
    last_digit_flag = 1 if input_array[-1] > 0.5 else 0
    return first_six, last_digit_flag
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
    chunk=50
    enc_layers = 4
    dec_layers = 7
    nheads = 8
    state_dim=7
    
    lr_backbone = 1e-5
    backbone = 'resnet18'
    policy_config = {'lr': 1e-5,
                         'num_queries': chunk,
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
    
    
   
    follower = ElephantRobot("192.168.3.223", 5001)
    follower.start_client()
    max_timesteps=2000
    query_frequency=chunk
    all_time_actions = torch.zeros([max_timesteps, max_timesteps+query_frequency, state_dim]).cuda()
    cap = Realsense2(camera_id_list=[0,1], camera_width=640, camera_height=480) # 
    cap.camera_config()
    
    with torch.inference_mode():
        for t in range(max_timesteps):
            
            cap.wait_frames()
            top = cap.rgb_image(0)
            wri= cap.rgb_image(1)
            cv2.imshow("top", top)
            cv2.imshow("wrist", wri)
            if cv2.waitKey(1) == ord("q"):
                break
            color_images=[]
            top= top.transpose((2, 0, 1))
            wri=wri.transpose((2, 0, 1))
            color_images.append(top)
            color_images.append(wri)
            curr_image = np.stack(color_images, axis=0)
            curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
            qpos_raw=np.array(follower.get_angles())
            if ISOPEN:
                qpos_raw=np.append(qpos_raw,0)
            else:
                qpos_raw=np.append(qpos_raw,1)
            qpos = pre_process(qpos_raw)
            qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
            
            if t % query_frequency == 0:
                all_actions = policy(qpos, curr_image)
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
            actionr,actiong=last_process(action)
            print(actiong)
            if actiong==1:
                close_g()
            else:
                open_g()
            follower.write_angles(actionr,5999)
            follower.command_wait_done()
           
            
            

    



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
