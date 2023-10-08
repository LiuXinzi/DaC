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

def clear_jpg_files(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".jpg"):
            filepath = os.path.join(directory, filename)
            os.remove(filepath)
            print(f"Deleted: {filename}")

# clean the content of photo and rec
episode_idx=49
camera1_id = 0  # 第一个相机的ID
camera2_id = 1  # 第二个相机的ID
save_folder_top = "./top"  # 第一个相机保存路径
save_folder_wrist = "./wrist"  # 第二个相机保存路径
os.makedirs(save_folder_top, exist_ok=True)
os.makedirs(save_folder_wrist, exist_ok=True)
clear_jpg_files('top')
clear_jpg_files('wrist')
log_file_path = 'rec.log'  # 将此路径替换为您的log文件路径
with open(log_file_path, 'w') as log_file:
    log_file.truncate(0)


# initial both camera
pipeline1 = rs.pipeline()
config1 = rs.config()
config1.enable_device(camera1_id)
pipeline1.start(config1)
pipeline2 = rs.pipeline()
config2 = rs.config()
config2.enable_device(camera2_id)
pipeline2.start(config2)

# initial robot and change the mode of print board
follower = ElephantRobot("10.24.179.109", 5001)
leader = ElephantRobot("10.25.161.59", 5001)
follower.start_client()
leader.start_client()
original_stdout = sys.stdout
sys.stdout = open('rec.log', mode = 'w',encoding='utf-8')

# cnt is the length of the data
CNTMAX=450
cnt = 0
data_dict = {
            '/observations/qpos': [],
            '/action': [],
        }
data_dict['/observations/images/top'] = []
data_dict['/observations/images/wrist'] = []


# start collect the data
while True:
        # 相机1捕获
        frames1 = pipeline1.wait_for_frames()
        color_frame1 = frames1.get_color_frame()
        color_image1 = np.asanyarray(color_frame1.get_data())

        # 相机2捕获
        frames2 = pipeline2.wait_for_frames()
        color_frame2 = frames2.get_color_frame()
        color_image2 = np.asanyarray(color_frame2.get_data())
        
        # 在OpenCV窗口中显示实时图像
        cv2.imshow("RealSense Camera 1", color_image1)
        cv2.imshow("RealSense Camera 2", color_image2)

        # 检测按键，如果按下'q'键则退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        cnt += 1
        # save the photo
        if True:
            filename_top = f"top/captured_image_{cnt}.jpg"
            cv2.imwrite(filename_top, color_image1)
            filename_wrist = f"wrist/captured_image_{cnt}.jpg"
            cv2.imwrite(filename_wrist, color_image2)
        # 将关节角数据添加到列表中
        leader.get_angles()
        follower.get_angles()
        if cnt>=CNTMAX:
             break

# stop collect the data
pipeline1.stop()
pipeline2.stop()
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


image_files_top = sorted([f for f in os.listdir(save_folder_top) if f.endswith('.jpg')])
# 遍历每个图像文件，将其读取并添加到数组中
for image_file in image_files_top:
    image_path = os.path.join(save_folder_top, image_file)
    img = Image.open(image_path)
    img = img.resize((640,480))  # 将图像调整为指定的大小（500x600）
    img_data = np.array(img)  # 将图像数据转换为NumPy数组
    data_dict['/observations/images/top'].append(img_data)
image_files_wrist = sorted([f for f in os.listdir(save_folder_wrist) if f.endswith('.jpg')])
# 遍历每个图像文件，将其读取并添加到数组中
for image_file in image_files_wrist:
    image_path = os.path.join(save_folder_wrist, image_file)
    img = Image.open(image_path)
    img = img.resize((640,480))  # 将图像调整为指定的大小（500x600）
    img_data = np.array(img)  # 将图像数据转换为NumPy数组
    data_dict['/observations/images/wrist'].append(img_data)

max_timesteps=len(data_dict['/action'])



dataset_path = f'/home/dc/python/sim_transfer_cube_human/{episode_idx}'
with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
        root.attrs['sim'] = True
        obs = root.create_group('observations')
        image = obs.create_group('images')
        
        _ = image.create_dataset('top', (max_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
        _ = image.create_dataset('wrist', (max_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
        qpos = obs.create_dataset('qpos', (max_timesteps, 6))
        action = root.create_dataset('action', (max_timesteps, 6))

        for name, array in data_dict.items():
            root[name][...] = array