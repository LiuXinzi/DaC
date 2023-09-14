import pyrealsense2 as rs
import cv2
import os
import numpy as np
from pymycobot import ElephantRobot


#############################
save_imgs = False
############################




if __name__ == "__main__":

    dir_imgs = "imgs_folder"
    dir_angles = "angles_folder"
    path_imgs = os.path.join(os.getcwd(), dir_imgs)
    path_angles = os.path.join(os.getcwd(), dir_imgs)

    if not os.path.exists(path_imgs):
        os.makedirs(path_imgs)
    if not os.path.exists(path_angles):
        os.makedirs(path_angles)
  

    # 连接机器人服务器
    elephant_client = ElephantRobot("192.168.137.182", 5001)

    # 开启TCP通信
    elephant_client.start_client()

    # 初始化RealSense摄像头
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 配置:  代码已经以30Hz的频率采集图像
    pipeline.start(config)


    cnt = 0
    print("!!!!!!!!!!!! Start Realsense !!!!!!!!!!!!")

    try:
        while True:
            cnt += 1

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

            if save_imgs:
                filename = f"{dir_imgs}/captured_image_{cnt}.jpg"
                cv2.imwrite(filename, color_image)

            # 得到关节角数据，需要为string
            angles = elephant_client.get_angles()
            

            # 拼接保存文件的路径和文件名
            angles_filepath = os.path.join(dir_angles, f"loop_{cnt}.txt")

            # 将结果写入保存文件
            with open(angles_filepath, "w") as f:
                f.write(angles)


    finally:
        print("!!!!!!!!!!!! Stop Realsense !!!!!!!!!!!!")
        
        # 关闭realsense的pipeline
        pipeline.stop()

        # 关闭opencv窗口
        cv2.destroyAllWindows()

        # 关闭大象的代理
        elephant_client.stop_client()


        # 将保存的txt进行合成，输出最终的txt，然后把循环中的txt都删掉
        save_filename = "saved_angels.txt"
        with open(save_filename, "w") as f:

            for i in range(cnt):
                # 拼接读取文件的路径和文件名
                read_filepath = os.path.join(dir_angles, f"loop_{i+1}.txt")

                # 读取文件内容
                with open(read_filepath, "r") as read_file:
                    content = read_file.read()

                # 将文件内容写入新文件
                f.write(content + "\n")

        # 删除之前保存的所有文件
        for i in range(cnt):
            filepath = os.path.join(dir_angles, f"loop_{i+1}.txt")
            os.remove(filepath)



     
