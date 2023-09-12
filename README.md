临期计划：
1. 角度和照片采集代码 9.12
2. 连接舵机
3. 无电机末端设计
4. 选择： 长按抓取 or 点按抓取

耗材：
1. 一根2m以上的传递照片的数据线
2. 控制夹爪器材正在列表
# 单一机械臂
## 遥控系统 --9.10左右完成
前六个关节通过代码实时读取数据，上传到follower同步角度。
  使用手柄控制第六个关节角度
  使用按钮控制夹爪开合
### 机械臂硬件及夹爪 --9.10
1. 使用胶带解决机械臂前三个关节不能自由移动的问题-- 胶带一天左右会松掉，可以在采集数据的时候上胶带 --完成
2. 夹爪设计以及3D打印 Qsj -- 完成
3. 使用robotis电机实现按钮控制电机运动 Gyt --本周
4. 摄像头安装 与标定 Qsj --完成
### 机械臂远程操控 --9.7 
大象机械臂操作过程：

leader机械臂-->python-get_angles-->python-write_angle-->socket-->树莓派-->控制follower机械臂

远程操作思路：由于socket到控制黑盒无法修改，使用每次循环等待运动完毕再进入下次循环

夹爪操作过程： 夹爪操作需要计算机传递信息给夹爪，目前分为两个部分
1. 按钮到计算机： 使用stm32单片机传递按钮01信号 --> 树莓派in 接口 --> 通过python 程序读取in接口状态确定闭合
2. 计算机到夹爪： 使用u2d2转接器通过USB连接树莓派 --> 通过u2d2转接器与舵机通信 --> 控制夹爪。

## 数据采集 --9.12左右完成
1. 安装两个机械臂并校准验机   --已完成
2. 划分任务区间与目标区域 --已完成
3. 实验区域标定：follower机械臂工作区间在黑色胶带范围内，黄色胶带区域可接触但不可垂直抓取（不适合用做物体摆放区域），预计抓取任务从黑色叉号标记点之间转移，摄像头在黑色胶带外，垂直照射工作区间的中心，并且标定中心为工作区间中心
4. 修改代码 zjw .13之前
5. 摄像头安装： 使用opencv读取视频流，目前数据线太短，需要较长的数据线
### 采集数据
action： leader 的joint position

observations: follower 的 joint position and picture

### 动作采集 --推动物体到达指定区域
具体动作： 夹爪推动快递盒，并将快递盒推动到规定黑色区域内

任务：使用摄像头获得动作照片，关节角度等信息并打包，采集50次用于后续训练
### 动作采集 --挂毛巾
具体任务： 夹爪抓取毛巾，放置于摆放好的架子之上

任务：使用摄像头获得动作照片，关节角度等信息并打包，采集50次用于后续训练

## 策略训练 --待数据采集完毕
### 代码修改
修改act代码，对接采集数据
### 策略训练
使用训练采集数据对act进行收敛训练
## 真机测试 --9.14-9.20
使用训练好的policy进行测试
