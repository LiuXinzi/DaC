import cv2
import os

# 设置输入图片文件夹和输出视频文件名
input_folder = '/home/dc/python/。。。。。。/saved_imgs_33'
output_video = 'output_video33.mp4'

# 获取文件夹中的所有图片文件名
images = [f'captured_image_{i}.jpg' for i in range(1,451)]

# 获取第一张图片的尺寸
frame = cv2.imread(os.path.join(input_folder, images[0]))
height, width, layers = frame.shape

# 设置视频的帧率和分辨率
fps = 30
video = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

# 逐帧将图片添加到视频中
for image in images:
    img_path = os.path.join(input_folder, image)
    frame = cv2.imread(img_path)
    video.write(frame)

# 释放视频对象
video.release()

print("视频合成完成：", output_video)
