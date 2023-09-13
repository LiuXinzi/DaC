#!/usr/bin/env python

import rospy
from pymycobot import ElephantRobot

import sys 
sys.path.append("..") 
# from leader_follower.msg import ArmJoints
from my_msgs.msg import ArmJoints

def start_leader_arm_publisher():
    rospy.init_node('leader_arm_publisher', anonymous=True)
    leader_arm = ElephantRobot("10.25.161.59", 5001)  # 创建leader机械臂对象，使用您的API来获取角度
    leader_arm.start_client() # 开启leader实例化对象

    leader_pub = rospy.Publisher('leader_arm_joints', ArmJoints, queue_size=1000)
    leader_rate = rospy.Rate(10)  # 设置发布频率，这里假设为10Hz

    while not rospy.is_shutdown():
        arm_joints = ArmJoints()
        arm_joints.joint1, arm_joints.joint2, arm_joints.joint3, arm_joints.joint4, arm_joints.joint5, arm_joints.joint6 = leader_arm.get_angles()  # 获取leader机械臂的角度
        

        leader_pub.publish(arm_joints)
        leader_rate.sleep()

if __name__ == '__main__':
    try:
        start_leader_arm_publisher()
    except rospy.ROSInterruptException:
        pass
