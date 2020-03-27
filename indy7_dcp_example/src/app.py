#!/usr/bin/python

import sys
import os
import math

from time import sleep
from std_msgs.msg import Bool
from moveit_msgs.msg import  DisplayRobotState
import rospy

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class JointControllerApp():
    def __init__(self):
        rospy.init_node("joint_controller")

        self.query_joint_state_pub = rospy.Publisher("/indy/query_joint_states", DisplayRobotState, queue_size=10)
        self.go_pub = rospy.Publisher("/indy/execute_plan", Bool, queue_size=1)
        self.stop_pub = rospy.Publisher("/indy/stop_robot", Bool, queue_size=1)

    def update_joint_pos(self, q):
        display_robot_state_msg = DisplayRobotState()
        display_robot_state_msg.state.joint_state.header.stamp = rospy.Time.now()
        display_robot_state_msg.state.joint_state.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        display_robot_state_msg.state.joint_state.position = degs2rads(q)
        display_robot_state_msg.state.joint_state.velocity = []
        display_robot_state_msg.state.joint_state.effort = []
        self.query_joint_state_pub.publish(display_robot_state_msg)

    def go(self):
        self.go_pub.publish(True)

    def stop(self):
        self.stop_pub.publish(True)

if __name__ == "__main__":
    app = JointControllerApp()
    
    while True:
        q = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            q[i] = float(raw_input('query joint {} position '.format(i)))

        print('joint query : ' + str(q))
        app.update_joint_pos(q)

        go_sign = raw_input('enter g to move robot ')
        if go_sign == 'g':
            app.go()
        else:
            continue
        
        stop_sign = raw_input('enter s to move robot ')
        if stop_sign == 's':
            app.stop()
        else:
            continue

        exit_sign = raw_input('enter q to quit program ')
        if exit_sign == 'q':
            break
        else:
            continue
        
    