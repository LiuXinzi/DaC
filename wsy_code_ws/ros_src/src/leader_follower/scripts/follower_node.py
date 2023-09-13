#!/usr/bin/env python

import rospy
from pymycobot import ElephantRobot
from my_msgs.msg import ArmJoints


def follower_arm_subscriber():
    rospy.init_node('follower_arm_subscriber', anonymous=True)
    follower_arm = ElephantRobot("10.24.179.109", 5001)  # 创建follower机械臂对象，使用您的API来设置角度
    follower_arm.start_client()

    def callback(data):
        joint1 = data.joint1
        joint2 = data.joint2
        joint3 = data.joint3
        joint4 = data.joint4
        joint5 = data.joint5
        joint6 = data.joint6
    
        follower_arm.write_angles([joint1, joint2, joint3, joint4, joint5, joint6], 1800)

        print(111)
        

    # 订阅leader机械臂发布的角度话题
    rospy.Subscriber('leader_arm_joints', ArmJoints, callback)  # LeaderArmMsg替换为实际的消息类型

    rospy.spin()

if __name__ == '__main__':
    try:
        follower_arm_subscriber()
    except rospy.ROSInterruptException:
        pass
