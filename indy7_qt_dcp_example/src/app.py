#!/usr/bin/python

import sys
import os

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QSlider
from PyQt5.QtCore import QSize, Qt, pyqtSlot,pyqtSignal
from PyQt5.uic import loadUi

from time import sleep
from std_msgs.msg import Bool
from moveit_msgs.msg import  DisplayRobotState
import rospy
import math

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads


class JointControllerApp(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi('/home/gwkim/catkin_ws/src/indy-ros-examples/indy7_qt_dcp_example/src/app.ui', self)

        rospy.init_node("joint_controller")

        self.query_joint_state_pub = rospy.Publisher("/indy/query_joint_states",  DisplayRobotState, queue_size=10)
        self.go_pub = rospy.Publisher("/indy/go_robot", Bool, queue_size=1)
        self.stop_pub = rospy.Publisher("/indy/stop_robot", Bool, queue_size=1)

        self.joint1_slide.setSingleStep(1)
        self.joint1_slide.setRange(-1750, 1750)
        self.joint1_slide.setValue(0)
        self.joint1_slide.valueChanged.connect(self.updateJointPos)

        self.joint2_slide.setSingleStep(1)
        self.joint2_slide.setRange(-1750, 1750)
        self.joint2_slide.setValue(0)
        self.joint2_slide.valueChanged.connect(self.updateJointPos)
        
        self.joint3_slide.setSingleStep(1)
        self.joint3_slide.setRange(-1750, 1750)
        self.joint3_slide.setValue(0)
        self.joint3_slide.valueChanged.connect(self.updateJointPos)

        self.joint4_slide.setSingleStep(1)
        self.joint4_slide.setRange(-1750, 1750)
        self.joint4_slide.setValue(0)
        self.joint4_slide.valueChanged.connect(self.updateJointPos)

        self.joint5_slide.setSingleStep(1)
        self.joint5_slide.setRange(-1750, 1750)
        self.joint5_slide.setValue(0)
        self.joint5_slide.valueChanged.connect(self.updateJointPos)
        
        self.joint6_slide.setSingleStep(1)
        self.joint6_slide.setRange(-1750, 1750)
        self.joint6_slide.setValue(0)
        self.joint6_slide.valueChanged.connect(self.updateJointPos)

        self.go_button.clicked.connect(self.goButtonClicked)
        self.stop_button.clicked.connect(self.stopButtonClicked)

    def updateJointPos(self):
        self.q = [
            self.joint1_slide.value()/10.0,
            self.joint2_slide.value()/10.0,
            self.joint3_slide.value()/10.0,
            self.joint4_slide.value()/10.0,
            self.joint5_slide.value()/10.0,
            self.joint6_slide.value()/10.0,
        ]

        self.joint1_val.setNum(self.q[0])
        self.joint2_val.setNum(self.q[1])
        self.joint3_val.setNum(self.q[2])
        self.joint4_val.setNum(self.q[3])
        self.joint5_val.setNum(self.q[4])
        self.joint6_val.setNum(self.q[5])


        display_robot_state_msg = DisplayRobotState()
        display_robot_state_msg.state.joint_state.header.stamp = rospy.Time.now()
        display_robot_state_msg.state.joint_state.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        display_robot_state_msg.state.joint_state.position = degs2rads(self.q)
        display_robot_state_msg.state.joint_state.velocity = []
        display_robot_state_msg.state.joint_state.effort = []


        self.query_joint_state_pub.publish(display_robot_state_msg)

    def goButtonClicked(self):
        self.go_pub.publish(True)

    def stopButtonClicked(self):
        self.stop_pub.publish(True)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = JointControllerApp()
    mainWin.show()
    sys.exit( app.exec_() )
