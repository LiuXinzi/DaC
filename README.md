# 单一机械臂
## 数据采集 --9.20左右完成
1. 安装两个机械臂并校准验机   --已完成
2. 安装电脑linux环境以及配置代码环境    --已完成
   
数据采集思路：

原文思路：

仿真环境之中 hdf5 封装 action observation， observation从env中task的get_observation中获得

现实环境中需要使用相机获得照片，使用代码获得角度和速度数剧打包成hdf5

action： 下一步or 当前状态 or 差值
## 遥控系统 --9.20左右完成
1. 上三个关节无法自由移动  --使用胶带来固定按钮
2. 购买follower夹爪 --已购买

遥控系统以及leader夹爪方案：

原文思路：

前六个关节通过代码实时读取数据，上传到follower同步角度。
夹爪关节最后通过数据同步读取，前爪连接另外一个电机，可以获得电机角度，之后和前六个关节方式相同。

项目思路：

前六个关节角度与代码相同，通过读取代码同步。但是前爪关节需要有变化，我们的前爪无电机--使用按钮控制，外接arduino，通过arduino与主机通讯传回夹爪开闭的01信号

1. 原文夹爪开合控制
2. 控制 使用其他语言
3. 夹爪末端
4. 实时同步

6. 
