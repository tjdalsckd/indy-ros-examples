#!/usr/bin/python

import sys
import os

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel
from PyQt5.QtCore import Qt
from PyQt5.uic import loadUi

from std_msgs.msg import Bool, Empty
import rospy

class MoveitControllerApp(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi('/home/gwkim/catkin_ws/src/indy-ros-examples/indy7_moveit_dcp_example/src/app.ui', self)

        rospy.init_node("moveit_controller")

        self.execute_pub = rospy.Publisher("/indy/execute_plan", Bool, queue_size=1)
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
        self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty, queue_size=5)
        self.stop_pub = rospy.Publisher("/indy/stop_robot", Bool, queue_size=1)

        self.execute_button.clicked.connect(self.executeButtonClicked)
        self.stop_button.clicked.connect(self.stopButtonClicked)
        self.plan_button.clicked.connect(self.planButtonClicked)

    def executeButtonClicked(self):
        self.execute_pub.publish(True)

    def stopButtonClicked(self):
        self.stop_pub.publish(True)

    def planButtonClicked(self):
        self.update_start_state_pub.publish(Empty())
        self.plan_pub.publish(Empty())


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MoveitControllerApp()
    mainWin.show()
    sys.exit( app.exec_() )
