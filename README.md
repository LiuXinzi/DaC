# 单一机械臂

## 遥控系统 --9.10左右完成
前六个关节通过代码实时读取数据，上传到follower同步角度。
  使用手柄控制第六个关节角度
  使用按钮控制夹爪开合
### 机械臂硬件及夹爪 --9.10
1. 使用胶带解决机械臂前三个关节不能自由移动的问题 Qsj
2. 夹爪设计以及3D打印 Qsj
3. 使用按钮连接arduino 控制夹爪闭合 Gyt
4. 摄像头安装 Qsj
### 机械臂远程操控 --9.7 
1. 编写python程序连接ros，进行线程监控前六个关节运动 Wsy
2. 测试程序

## 数据采集 --9.12左右完成
1. 安装两个机械臂并校准验机   --已完成
2. 安装电脑linux环境以及配置代码环境    --已完成
### 动作采集 --物体抓取
使用摄像头获得动作照片，关节角度等信息并打包，采集50次用于后续训练
### 动作采集 --挂毛巾
使用摄像头获得动作照片，关节角度等信息并打包，采集50次用于后续训练

## 策略训练 --9.12-9.14
### 代码修改
修改act代码，对接采集数据
### 策略训练
使用训练采集数据对act进行收敛训练
## 真机测试 --9.14-9.20
使用训练好的policy进行测试
