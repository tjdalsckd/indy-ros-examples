#!/usr/bin/python

import sys
import math
import json

from indydcp import indydcp_client
from indydcp import indy_program_maker

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool, Float64MultiArray
from moveit_msgs.msg import MoveGroupActionResult, DisplayRobotState
from geometry_msgs.msg import Pose

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class IndyROSConnector:
    def __init__(self):

        _bind_ip = "192.168.3.104"
        _robot_ip = "192.168.3.117"
        _name = "NRMK-Indy7"
       
        # Connect
        self.indy = indydcp_client.IndyDCPClient(_bind_ip, _robot_ip, _name)
        self.indy.connect()

        rospy.init_node('indy_driver_py')
        self.rate = rospy.Rate(50) # hz

        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.go_sub = rospy.Subscriber("/indy/go_robot", Bool, self.goRobotCallBack, queue_size=1)
        self.stop_sub = rospy.Subscriber("/indy/stop_robot", Bool, self.stopRobotCallBack, queue_size=1)
        self.query_joint_state_sub = rospy.Subscriber("/indy/query_joint_states", DisplayRobotState,  self.jointStateCallBack, queue_size=10)
        self.q = [0, 0, 0, 0, 0, 0]

        self.isExecuted = False

    def jointStateCallBack(self, msg):
        self.q = rads2degs(msg.state.joint_state.position)
        
    def goRobotCallBack(self, msg):
        self.isExecuted = True

    def go(self):
        prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
            
        prog.add_joint_move_to(self.q, vel=1, blend=0)
        
        json_string = json.dumps(prog.json_program)
        self.indy.set_and_start_json_program(json_string)

    def jointStatePub(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        joint_state_msg.position = degs2rads(self.indy.get_joint_pos())
        joint_state_msg.velocity = []
        joint_state_msg.effort = []

        self.joint_state_pub.publish(joint_state_msg)
    
    def stopRobotCallBack(self, msg):
            self.indy.stop_motion()
            
    def __del__(self):
        self.indy.disconnect()

    def run(self):
        while not rospy.is_shutdown():
            if self.isExecuted:
                self.go()
                self.isExecuted = False
            else:
                self.jointStatePub()
            self.rate.sleep()

def main():
    t = IndyROSConnector()
    t.run()
                
if __name__ == '__main__':
    main()