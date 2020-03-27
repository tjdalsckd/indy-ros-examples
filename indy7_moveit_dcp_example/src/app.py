#!/usr/bin/python

import sys
import os

from std_msgs.msg import Bool, Empty
import rospy

class MoveitControllerApp():
    def __init__(self):
        rospy.init_node("moveit_controller")

        self.execute_pub = rospy.Publisher("/indy/execute_plan", Bool, queue_size=1)
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
        self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty, queue_size=5)
        self.stop_pub = rospy.Publisher("/indy/stop_robot", Bool, queue_size=1)

    def publish_execute(self):
        self.execute_pub.publish(True)

    def publish_stop(self):
        self.stop_pub.publish(True)

    def publish_plan(self):
        self.update_start_state_pub.publish(Empty())
        self.plan_pub.publish(Empty())


if __name__ == "__main__":
    app = MoveitControllerApp()
    
    while True:
        sign = raw_input('enter p (planning) | s (stop) | e (execute) | q (quit)')
        if sign == 'p':
            app.publish_plan()
        elif sign == 's':
            app.publish_stop()
        elif sign == 'e':
            app.publish_execute()
        elif sign == 'q':
            break
