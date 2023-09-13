import pyrealsense2 as rs
import cv2
import numpy as np
from pymycobot import ElephantRobot


save_imgs = False




# 创建一个空的列表来存储关节角数据
angles_list = []

# "连接机器人服务器"
elephant_client = ElephantRobot("192.168.137.182", 5001)

# "开启TCP通信"
elephant_client.start_client()


# 初始化RealSense摄像头
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 配置:  代码已经以30Hz的频率采集图像
pipeline.start(config)

images_to_save = []

cnt = 0
print("!!!!!!!!!!!! Start Realsense !!!!!!!!!!!!")


try:
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

        # images_to_save.append(color_image)
        cnt += 1

        if save_imgs:
            filename = f"saved_imgs/captured_image_{cnt}.jpg"
            cv2.imwrite(filename, color_image)

        # 将关节角数据添加到列表中
        angles_list.append(elephant_client.get_angles())




finally:
    print("!!!!!!!!!!!! Stop Realsense !!!!!!!!!!!!")
    pipeline.stop()
    cv2.destroyAllWindows()

    elephant_client.stop_client()



    # 退出循环后，将关节角数据保存到txt文件
    with open('joint_angles.txt', 'w') as file:
        for angles_data in angles_list:
            file.write(angles_data + '\n')
