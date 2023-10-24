eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCIsImtpZCI6Ik1UaEVOVUpHTkVNMVFURTRNMEZCTWpkQ05UZzVNRFUxUlRVd1FVSkRNRU13UmtGRVFrRXpSZyJ9.eyJodHRwczovL2FwaS5vcGVuYWkuY29tL3Byb2ZpbGUiOnsiZW1haWwiOiIyNTk4MzMwMjk3QHFxLmNvbSIsImVtYWlsX3ZlcmlmaWVkIjp0cnVlfSwiaHR0cHM6Ly9hcGkub3BlbmFpLmNvbS9hdXRoIjp7InBvaWQiOiJvcmctb1J0cXZxNkJtWUhKTGJ2ckdyeHY4cjBiIiwidXNlcl9pZCI6InVzZXItanMxUllRMzRrYUVidGF6Y3dYNU5HeUN2In0sImlzcyI6Imh0dHBzOi8vYXV0aDAub3BlbmFpLmNvbS8iLCJzdWIiOiJhdXRoMHw2M2U5ZTdiMTc0Zjk0YWM1MmJjNzIxYTkiLCJhdWQiOlsiaHR0cHM6Ly9hcGkub3BlbmFpLmNvbS92MSIsImh0dHBzOi8vb3BlbmFpLm9wZW5haS5hdXRoMGFwcC5jb20vdXNlcmluZm8iXSwiaWF0IjoxNjk4MTI2NDQwLCJleHAiOjE2OTg5OTA0NDAsImF6cCI6IlRkSkljYmUxNldvVEh0Tjk1bnl5d2g1RTR5T282SXRHIiwic2NvcGUiOiJvcGVuaWQgcHJvZmlsZSBlbWFpbCBtb2RlbC5yZWFkIG1vZGVsLnJlcXVlc3Qgb3JnYW5pemF0aW9uLnJlYWQgb3JnYW5pemF0aW9uLndyaXRlIG9mZmxpbmVfYWNjZXNzIn0.szpe-PNt1q1QN-bthoYlUuN3OmwLHRLU6PTnbQE55NKl86jh6P_5jrY2cJXl_nIC73zK_wS69mrdSB6zvQgCkPK0vJ0Dxgrhp9Ej16kCqT-rL7-0LTMbUijkQcGooq870fjxM7cFUt_ISJ58Gd0vTFXTXkoJEsmS5NWR4WMj73fjym3iDG3u_HPmx06nRNr6Of7e8T7HrJtl6wxfMI4CaWzThPMVY__TsycEPYOcdpRCAO6IOLaDr5T8bRpV6bmhmiJdyMyyXvwXHhccGjnz6oIzHaT9nbp38xiaw3zkbQ-WPlzuCfXhkpycSO7XJU6tFO6gIaoKZpy71G8AwyGBFg
## 计划（10.23-10.29）
- 通信尝试进一步优化
- 通过模仿学习实现抓取任务（对于0-1夹爪）
- 实现连续夹爪功能
# 单一机械臂控制
## 遥控系统 
### 原文的部署与遥控系统
1. ACT原文使用WindowX遥控操作关节更大的ViperX机械臂，其余的设置包括一个机器人笼，通过交叉钢电缆加固。总共有四个罗技C922x网络摄像头，每个设置480×640RGB图像。其中两个网络摄像头安装在跟随者机器人的手腕上，可以近距离观察抓持器。其余的两个摄像机分别安装在前面和顶部。远程操作和数据记录频率都为50HZ。\
2. 原文考虑过使用VR控制器或摄像机捕捉到的手姿势映射到机器人的末端执行器姿势，即任务空间映射，最后没有采用的原因是精细操作需要在奇点处进行，反向运动学会出错。另一方面，联合空间映射保证了在联合限制范围内的高带宽控制，同时也需要更少的计算和减少延迟。原文只提及到通过关节角度同步来进行远程操控，但是根据视频可以看出关节同步效果好，延迟低。
### 目前的部署(等待优化)
1. 设备部署：
    使用两台大象pro600机械臂，位于操作者面向方向，摄像头位于操作者左手边，正中间为机械臂工作区间，follower机械臂旁边搭配一盏台灯，可以有效消除上方灯光影响，leader机械臂上方用胶带捆绑一硬质物体用来保持自由移动
2. 末端执行器：
   leader机械臂采用手柄加按钮，手柄为3d打印材料，按钮为自锁按钮。follower机械臂末端执行器目前有两种，用来完成推动任务的长木棍末端，和用来抓取的夹爪末端。

### 前六个关节的遥控通路与通讯频率
#### 通路
前六个关节角度：leader机械臂-->python-get_angles-->python-write_angle-->socket-->树莓派-->控制follower机械臂
#### 频率
本次测试主要使用三个时间作为频率数据
1. 读取时间： 指的是a机械臂角度的读取时间 使用wifi 在0.0015-0.002s 约 500-660HZ 使用网线没有较大变化
2. 通信时间： 指的是a机械臂角度读取到传递给b机械臂的时间 使用wifi在0.006-0.01s 约 100-167HZ 使用网线在0.003-0.0015s 在330-660HZ
3. 控制所需时间： 指的是b机械臂得到角度之后到开始控制需要的时间 使用wifi在0.005s左右 使用网线在0.001s\
使用wifi在静止的时候（忽略运动所需时间）：每一个while循环（读取通信加控制传递）在0.01s左右 使用网线在0.003-0.004s 通信速度变快
机械臂在得到角度之后需要跟随运动，最大运动速度在1999（机械臂速度参数最大）的时候，操作较慢的情况下，跟随运动在0.4s左右，操作较快需要跟随1s左右，不运动的时候时间可以忽略，这个时间目前使用command_wait_done函数去中和掉，也可以使用time.sleep(o.35)去等待运动\
在数据传输过程中（不用command_wait）可能会有数据传输过去被覆盖，可以用通信队列尝试优化（未尝试）
### 末端的控制通路
   夹爪（第七个电机）电机型号为：DYNAMIXEL XH540-W150（文章里面是基于DYNAMIXEL XL430-W250）  通路为leader机械臂树莓派感知端口变化--->u2d2-->舵机运动


## 算法原理&真机部署
### 归一化
![image](https://github.com/LiuXinzi/DaC/assets/133741133/5700cd72-b7fb-4697-8dcc-4a24279898ee)\
qpos为采集的五十组数据里所有follower的角度的平均值和方差\
action为采集的五十组数据里所有leader的角度的平均值和方差\
pre是训练之前做的操作，post是网络输出角度后做的操作
### 算法原理
使用硬件收集用于测试的演示数据，将leader机械臂作为actions，observations由follower的关节位置与摄像机的图像传输组成。接下来根据当前的观察结果来预测未来的actions。这里的一个动作对应于下一个时间步中臂的目标关节位置。假设chunk的大小固定为k：每一个k个步骤都会收到一个观察结果，生成下一个k个操作，并依次执行这些操作。意味着任务的有效范围减少了k倍\
Temporal Ensemble：为了避免一个新的环境被突然合并带来的剧烈运动，让不同的动作块重叠，通过指数加权来预测\
网络包括一个一个CVAE编码器和一个CVAE解码器。CVAE编码器：仅用于训练CVAE解码器，测试的时候不需要。输入为当前的观察和动作序列作，预测样式变量z的分布的均值和方差，它被参数化为一个对角高斯分布。CVAE解码器：即策略，使用z条件和当前观测（图像+联合位置）来预测动作序列。在测试时将z设为先验分布的平均值，即从0到确定性解码。\
参数：\
learning rate 1e-5, batch size 8, encode layers 4, decoder layers 7, feedforward dimension 3200, hidden dimension 512, heads 8, chunk size 100 论文提及size为100的时候效果最好, beta 10, dropout 0.1
### 原文数据采集
原文使用ALOHA远程操作来收集演示。根据任务的复杂性，每次需要8-14秒，控制频率为50Hz，400-700个时间步长，为每个任务记录了50个演示，除了螺纹尼龙搭扣有100个。搭载四个摄像机，两个机械臂，收集以下信息用于训练：

action sequence： 14维，（joint position + 0/1 ）*2

observations joints： 14维，（joint position + 0/1 ）*2

observations 4 RGB images： 4 * 480* 640 * 3
### 推木块数据采集
目前使用树莓派互相通信遥控机械臂，以30HZ的频率收集拍照，使用大象的python api写入leader和follower的关节角度到rec.log文件中，读取文件使用两个list来保存两个机械臂分别的关节角度，之后与image一起打包为一组hdf5文件

action sequence： 6维，joint position

observations joints： 6维，joint position

observations top and wrist RGB images： 2 * 480* 640 * 3

目前暂时不采用夹爪，暂时先依旧采取无电机末端，结合两个摄像头的数据和更好的摇操来获得更好的数据，尝试效果，如果还是不行估计可能就是算法不能直接只改输入维度。\
我经过测试，给leader划分了一个类似的区间，希望可以在摇操的时候，不用等待follower到达，来获得更好的效果
### 01夹爪数据采集
目前使用树莓派互相通信遥控机械臂，以30HZ的频率收集拍照，使用大象的python api写入leader和follower的关节角度到rec.log文件中，读取文件使用两个list来保存两个机械臂分别的关节角度，之后与image一起打包为一组hdf5文件

action sequence： 7维，joint position + 01

observations joints： 7维，joint position + 01

observations top and wrist RGB images： 2 * 480* 640 * 3



