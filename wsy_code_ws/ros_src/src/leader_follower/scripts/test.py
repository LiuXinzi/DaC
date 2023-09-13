from pymycobot import ElephantRobot
import time
import numpy as np

leader_arm = ElephantRobot("10.25.161.59", 5001)
leader_arm.start_client()

follower_arm = ElephantRobot("10.24.179.109", 5001)  # 创建follower机械臂对象，使用您的API来设置角度
follower_arm.start_client()


time_get = 0
cnt = 0
cnt_max = 100
while cnt <= cnt_max:
   


    

    


    
    print()