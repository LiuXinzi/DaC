# 单一机械臂

## 遥控系统 --9.10左右完成
前六个关节通过代码实时读取数据，上传到follower同步角度。
  使用手柄控制第六个关节角度
  使用按钮控制夹爪开合
### 机械臂硬件及夹爪 --9.10
1. 使用胶带解决机械臂前三个关节不能自由移动的问题-- 胶带一天左右会松掉，可以在采集数据的时候上胶带 --完成
2. 夹爪设计以及3D打印 Qsj -- 完成
3. 使用按钮连接arduino 控制夹爪闭合 Gyt --本周
4. 摄像头安装 Qsj --已购买
### 机械臂远程操控 --9.7 
1. 使用python while循环调用大象api --卡顿
2. 使用ros1 使用双线程控制机械臂 --延迟性过高
3. 使用python 多线程 --卡顿

大象机械臂操作过程：

leader机械臂-->python-get_angles-->python-write_angle-->socket-->树莓派-->控制follower机械臂

解决方案1 ：由于socket到控制黑盒无法修改，将每次循环停顿0.05秒改为每次循环等待运动完毕再进入下次循环

## 数据采集 --9.12左右完成
1. 安装两个机械臂并校准验机   --已完成
2. 划分任务区间与目标区域 --9.8
### 动作采集 --物体抓取与放置
具体动作： 夹爪夹取快递盒，并将快递盒放置在规定黑色区域内

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
