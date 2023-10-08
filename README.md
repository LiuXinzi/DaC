eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCIsImtpZCI6Ik1UaEVOVUpHTkVNMVFURTRNMEZCTWpkQ05UZzVNRFUxUlRVd1FVSkRNRU13UmtGRVFrRXpSZyJ9.eyJodHRwczovL2FwaS5vcGVuYWkuY29tL3Byb2ZpbGUiOnsiZW1haWwiOiIyNTk4MzMwMjk3QHFxLmNvbSIsImVtYWlsX3ZlcmlmaWVkIjp0cnVlfSwiaHR0cHM6Ly9hcGkub3BlbmFpLmNvbS9hdXRoIjp7InVzZXJfaWQiOiJ1c2VyLWpzMVJZUTM0a2FFYnRhemN3WDVOR3lDdiJ9LCJpc3MiOiJodHRwczovL2F1dGgwLm9wZW5haS5jb20vIiwic3ViIjoiYXV0aDB8NjNlOWU3YjE3NGY5NGFjNTJiYzcyMWE5IiwiYXVkIjpbImh0dHBzOi8vYXBpLm9wZW5haS5jb20vdjEiLCJodHRwczovL29wZW5haS5vcGVuYWkuYXV0aDBhcHAuY29tL3VzZXJpbmZvIl0sImlhdCI6MTY5NjA1MjQ2NSwiZXhwIjoxNjk2OTE2NDY1LCJhenAiOiJwZGxMSVgyWTcyTUlsMnJoTGhURTlWVjliTjkwNWtCaCIsInNjb3BlIjoib3BlbmlkIHByb2ZpbGUgZW1haWwgbW9kZWwucmVhZCBtb2RlbC5yZXF1ZXN0IG9yZ2FuaXphdGlvbi5yZWFkIG9mZmxpbmVfYWNjZXNzIn0.mG0GOnCtiuWIvI2urpgm3SC2KGvc3UIxRQJzhCa7Dd7FkmFhHMJ-k8R7-FDWJJL7F29-IqiwvQQ0Ywxazw3qStct1gmIsRlte31vAv2dMp43csOWqWrd_bcy5PYzOwVxPCKQ6wUfe1oFs14MKL7S9LXNse04ffdhRLp8F57NulfxPdQahkPrCoxBCLYyOHXa4oX8blBnAQj-GLWZNuegBuTET8cP61hb6rrttrSqCf_ZxO1wR-dAmo5ZbWr-7cAyn6NE0hLowbWLOgfrztsGGj-XE85BiyZqH34DsaJj0Dq_H2f3ZHXeAOJzkh_jLrOb9ecvN3N-cbiKPlFNelDYuw

## act 介绍
数据 ： 链接：https://pan.baidu.com/s/1-KHDPKEF_zEDTcvXlYQeXA 
提取码：1234\
数据保存在sim_transfer_cube_scripted文件夹中

代码 imitate_episodes 是原代码训练测试程序，我们的测试程序为try.py，装配完环境之后运行原文代码把文件改为try就可以

运行：
python3 try.py \
--task_name sim_transfer_cube_scripted \
--ckpt_dir <ckpt dir> \
--policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
--num_epochs 2000  --lr 1e-5 \
--seed 0
# 单一机械臂控制
## 遥控系统 
### 原文&目前-思路对比
#### 原文的部署与遥控系统
1. ACT原文使用WindowX遥控操作关节更大的ViperX机械臂，其余的设置包括一个机器人笼，通过交叉钢电缆加固。总共有四个罗技C922x网络摄像头，每个设置480×640RGB图像。其中两个网络摄像头安装在跟随者机器人的手腕上，可以近距离观察抓持器。其余的两个摄像机分别安装在前面和顶部。远程操作和数据记录频率都为50HZ。

2. 原文考虑过使用VR控制器或摄像机捕捉到的手姿势映射到机器人的末端执行器姿势，即任务空间映射，最后没有采用的原因是精细操作需要在奇点处进行，反向运动学会出错。另一方面，联合空间映射保证了在联合限制范围内的高带宽控制，同时也需要更少的计算和减少延迟。原文只提及到通过关节角度同步来进行远程操控，但是根据视频可以看出关节同步效果好，延迟低。
#### 目前的部署与遥控系统
1. 设备部署：
   
    使用两台大象pro600机械臂，位于操作者面向方向，摄像头位于操作者左手边，正中间为机械臂工作区间（），follower机械臂旁边搭配一盏台灯，可以有效消除上方灯光影响（），leader机械臂上方用胶带捆绑一硬质物体用来保持自由移动
2. 末端执行器：
   
   leader机械臂采用手柄加按钮，手柄为3d打印材料，按钮为自锁按钮。follower机械臂末端执行器目前有两种，用来完成推动任务的长木棍末端，和用来抓取的夹爪末端。
   
3. 遥控系统：
   
   前六个关节角度：leader机械臂-->python-get_angles-->python-write_angle-->socket-->树莓派-->控制follower机械臂，读取需要时间，运动也需要时间，但是读取和运动并不同步。（即读取完并传递信号给follower机械臂为程序内进程，此时程序结束，之后树莓派控制机械臂不属于程序内，就会导致可叠加的延迟）因此如果只用while循环，不断进入循环会导致延迟不断增加，所以同步过程修改代码为等待程序运行完毕之后进入下一个while循环，即等待树莓派控制机械臂完成（动作彻底完成）再获得下一个动作。

   夹爪（第七个电机）： 通过计算机通过leader机械臂的树莓派in端口的点位变化来控制舵机的旋转。首先按钮到计算机,线路为 树莓派GND---自锁按钮---树莓派in端口，当自锁按钮按下，电路导通，in端口获得1的输入，按钮打开线路断开，in端口状态转0。 之后计算机到舵机，大象机械臂带有读取树莓派in端口的api，计算机重复调用方法读取端口，当端口状态变化后连接u2d2旋转舵机指定角度。最后舵机到夹爪，夹爪通过3d打印，预留固定舵机的孔位，将舵机的砝码盘固定在夹爪主动件的圆盘上，舵机转动带动主动盘转动，之后带动连接件以及夹爪末端开合。

两个机械臂虽然每次启动都会调零，但是在实现中发现，当机械臂移动了一段时间之后，角度会有区别，如果这个问题可以解决的话，可以在leader机械臂旁边放置一块大小相似的物体，直接用leader去推动物体完成动作，这样也可以解决延迟问题（移动后需要等待才能判断follower机械臂是否移动到适当位置）。

## 算法原理&真机部署
### 算法原理
使用硬件收集用于测试的演示数据，将leader机械臂作为actions，observations由follower的关节位置与摄像机的图像传输组成。接下来根据当前的观察结果来预测未来的actions。这里的一个动作对应于下一个时间步中臂的目标关节位置。假设chunk的大小固定为k：每一个k个步骤都会收到一个观察结果，生成下一个k个操作，并依次执行这些操作。意味着任务的有效范围减少了k倍

Temporal Ensemble：为了避免一个新的环境被突然合并带来的剧烈运动，让不同的动作块重叠，通过指数加权来预测

网络包括一个一个CVAE编码器和一个CVAE解码器。CVAE编码器：仅用于训练CVAE解码器，测试的时候不需要。输入为当前的观察和动作序列作，预测样式变量z的分布的均值和方差，它被参数化为一个对角高斯分布。CVAE解码器：即策略，使用z条件和当前观测（图像+联合位置）来预测动作序列。在测试时将z设为先验分布的平均值，即从0到确定性解码。

参数：

learning rate 1e-5, batch size 8, encode layers 4, decoder layers 7, feedforward dimension 3200, hidden dimension 512, heads 8, chunk size 100 论文提及size为100的时候效果最好, beta 10, dropout 0.1
### 数据采集
#### 原文的数据收集
原文使用ALOHA远程操作来收集演示。根据任务的复杂性，每次需要8-14秒，控制频率为50Hz，400-700个时间步长，为每个任务记录了50个演示，除了螺纹尼龙搭扣有100个。搭载四个摄像机，两个机械臂，收集以下信息用于训练：

action sequence： 14维，（joint position + 0/1 ）*2

observations joints： 14维，（joint position + 0/1 ）*2

observations 4 RGB images： 4 * 480* 640 * 3
#### 目前的数据采集
目前使用syn遥控机械臂，以30HZ的频率收集拍照，使用大象的python api写入leader和follower的关节角度到rec.log文件中，读取文件使用两个list来保存两个机械臂分别的关节角度，之后与image一起打包为一组hdf5文件

action sequence： 6维，joint position

observations joints： 6维，joint position

observations top RGB images： 1 * 480* 640 * 3

目前暂时不采用夹爪，暂时先依旧采取无电机末端，结合两个摄像头的数据和更好的摇操来获得更好的数据，尝试效果，如果还是不行估计可能就是算法不能直接只改输入维度。\
我经过测试，给leader划分了一个类似的区间，希望可以在摇操的时候，不用等待follower到达，来获得更好的效果

#### 原因分析
1. 网络只修改了输入参数维度，可能需要调节内部结构（难度可能较大，我估计无法完成）
2. 数据流畅性，数据误差大，较多时候没有精准完成（），数据随机性大，训练任务较多
3. 摄像头信息不够？


