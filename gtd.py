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

#use to clean the jpg        
def clear_jpg_files(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".jpg"):
            filepath = os.path.join(directory, filename)
            os.remove(filepath)

#clean the datafile of the photo and the angle
def init_file():
    save_folder_top = "./top"  # 第一个相机保存路径
    save_folder_wrist = "./wrist"  # 第二个相机保存路径
    os.makedirs(save_folder_top, exist_ok=True)
    os.makedirs(save_folder_wrist, exist_ok=True)
    clear_jpg_files('top')
    clear_jpg_files('wrist')
    log_file_path = 'rec.log'  
    with open(log_file_path, 'w') as log_file:
        log_file.truncate(0)
    return save_folder_top,save_folder_wrist,log_file_path


if __name__ == '__main__':
    save_folder_top,save_folder_wrist,log_file_path=init_file()
    # some constants and data
    episode_idx=0
    CNTMAX=450
    cnt = 0
    data_dict = {
            '/observations/qpos': [],
            '/action': [],
            }
    data_dict['/observations/images/top'] = []
    data_dict['/observations/images/wrist'] = []
    


    #initial the robot and cap
    follower = ElephantRobot("10.24.179.109", 5001)
    leader = ElephantRobot("10.25.161.59", 5001)
    follower.start_client()
    leader.start_client()
    cap = Realsense2(camera_id_list=[0,1], camera_width=640, camera_height=480) # 
    cap.camera_config()
    
    #change the out type
    original_stdout = sys.stdout
    sys.stdout = open('rec.log', mode = 'w',encoding='utf-8')



    while True:
        cap.wait_frames()
        img0 = cap.rgb_image(0)
        img1 = cap.rgb_image(1)
        cv2.imshow("top", img0)
        cv2.imshow("wrist", img1)
        if cv2.waitKey(1) == ord("q"):
            break
        # write the photo into the filepath
        if True:
            filename_top = f"top/captured_image_{cnt}.jpg"
            cv2.imwrite(filename_top, img0)
            filename_wrist = f"wrist/captured_image_{cnt}.jpg"
            cv2.imwrite(filename_wrist, img1)
        # 将关节角数据添加到列表中
        leader.get_angles()
        follower.get_angles()
        cnt+=1
        if cnt>=CNTMAX:
             break
    #clean the process and clints
    cap.stop()
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
    dataset_path = f'/home/dc/python/test'


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