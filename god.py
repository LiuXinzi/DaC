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
# clean the content
episode_idx=49

folder_path = f'saved_imgs_{episode_idx}'
import os

# 定义要创建的文件夹的名称

import os

# 定义要创建的文件夹的名称


# 使用os.path.exists()检查文件夹是否存在
if not os.path.exists(folder_path):
    # 如果文件夹不存在，则创建它
    os.makedirs(folder_path)


log_file_path = 'rec.log'  # 将此路径替换为您的log文件路径
with open(log_file_path, 'w') as log_file:
    log_file.truncate(0)

# set the robot and others
pipeline = rs.pipeline()
config = rs.config()
# 配置:  代码已经以30Hz的频率采集图像
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  
pipeline.start(config)
follower = ElephantRobot("10.24.179.109", 5001)
leader = ElephantRobot("10.25.161.59", 5001)
follower.start_client()
leader.start_client()
original_stdout = sys.stdout
sys.stdout = open('rec.log', mode = 'w',encoding='utf-8')

cnt = 0
data_dict = {
            '/observations/qpos': [],
            '/action': [],
        }
data_dict['/observations/images/top'] = []

# start collect the data
while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        # 将RealSense摄像头捕获的图像数据转换为OpenCV格式
        color_image = np.asanyarray(color_frame.get_data())

        # 在OpenCV窗口中显示实时图像
        cv2.imshow('RealSense Camera', color_image)

        # 检测按键，如果按下'q'键则退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        cnt += 1
        if True:
            filename = f"saved_imgs_{episode_idx}/captured_image_{cnt}.jpg"
            cv2.imwrite(filename, color_image)
        # 将关节角数据添加到列表中
        leader.get_angles()
        follower.get_angles()
        if cnt>=450:
             break

# stop collect the data
pipeline.stop()
cv2.destroyAllWindows()
follower.stop_client()
leader.stop_client()
sys.stdout = original_stdout

# 打开.log文件以读取内容
with open('rec.log', 'r') as file:
    lines = file.readlines()  # 读取所有行到一个列表中
for i in range(len(lines)):
    line = lines[i]
    match = re.search(r'\[(.*?)\]', line)
    if match:
        array_str = match.group(1)
        array = [float(x) for x in array_str.split(',')]
        if i % 2 == 0:
            data_dict['/action'].append(array)
        else:
            data_dict['/observations/qpos'].append(array)


image_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.jpg')])
# 遍历每个图像文件，将其读取并添加到数组中
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    img = Image.open(image_path)
    img = img.resize((640,480))  # 将图像调整为指定的大小（500x600）
    img_data = np.array(img)  # 将图像数据转换为NumPy数组
    data_dict['/observations/images/top'].append(img_data)
max_timesteps=len(data_dict['/action'])





dataset_path = f'/home/dc/python/hdf5_set/{episode_idx}'
with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
        root.attrs['sim'] = True
        obs = root.create_group('observations')
        image = obs.create_group('images')
        
        _ = image.create_dataset('top', (max_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
        # compression='gzip',compression_opts=2,)
        # compression=32001, compression_opts=(0, 0, 0, 0, 9, 1, 1), shuffle=False)
        qpos = obs.create_dataset('qpos', (max_timesteps, 6))
        action = root.create_dataset('action', (max_timesteps, 6))

        for name, array in data_dict.items():
            root[name][...] = array