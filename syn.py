from pymycobot import ElephantRobot
import time
import threading

"连接机器人服务器"
leader = ElephantRobot("10.25.161.59", 5001)
follower = ElephantRobot("10.24.179.109", 5001)

follower.start_client()
leader.start_client()
inilist=[]
while True:
    start=time.time()
    a=leader.get_angles()
    middle=time.time()
    follower.write_angles(a,1900)
    end=time.time()
    print(middle-start,end-middle)
