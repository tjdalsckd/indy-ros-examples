#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  #"""
  #Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  #@param: goal       A list of floats, a Pose or a PoseStamped
  #@param: actual     A list of floats, a Pose or a PoseStamped
  #@param: tolerance  A float
  #@returns: bool
  #"""
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        #scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        group_name = "indy7"
        group = moveit_commander.MoveGroupCommander(group_name)
        hand_group = moveit_commander.MoveGroupCommander('robotiq')

        ##to change the color or collision objects, We publish planning scene msg.
        planning_scene_publisher = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene, queue_size=1)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame
        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        #print "============ End effector: %s" % eef_link
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        #print "============ Printing robot state"
        #print robot.get_current_state()
        #print ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        #self.scene = scene
        self.group = group
        self.hand = hand_group
        #self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_scene_publisher = planning_scene_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def grasp(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.hand

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.36

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_table(self, plscene_msg,timeout = 4):
        robot = self.robot
        box_name = self.box_name
        planning_scene_publisher = self.planning_scene_publisher

        collision_object_table = moveit_msgs.msg.CollisionObject()

        collision_object_table.header.frame_id = robot.get_planning_frame()
        collision_object_table.id = "table"
        collision_object_table.operation = collision_object_table.ADD

        #define SolidPrimitive message to append to collision_object_table
        table = shape_msgs.msg.SolidPrimitive()
        table.type = shape_msgs.msg.SolidPrimitive.BOX
        table.dimensions = [0.5,0.7,0.1]
        collision_object_table.primitives = [table]

        #define position message to append to collision_object_table
        pose = geometry_msgs.msg.Pose()
        pose.orientation.w = 1.0
        pose.position.x = 0.55
        pose.position.y = 0.0
        pose.position.z = 0.05
        collision_object_table.primitive_poses = [pose]

        #append collision_object_table to the planning scene msg.
        plscene_msg.world.collision_objects.append(collision_object_table)
        #plscene_msg.is_diff = True
        #and publish.
        return plscene_msg

    def add_cube(self, plscene_msg, desired_pose, timeout=4):
        planning_scene_publisher = self.planning_scene_publisher
        robot = self.robot

        collision_object_cube = moveit_msgs.msg.CollisionObject()
        collision_object_cube.header.frame_id = robot.get_planning_frame()
        collision_object_cube.id = "cube_{0}".format(len(plscene_msg.world.collision_objects))
        #As cubes are added into the world, its id will be named like cube_1, cube_2...
        collision_object_cube.operation = collision_object_cube.ADD

        #define SolidPrimitive message to append to collision_object_table
        cube = shape_msgs.msg.SolidPrimitive()
        cube.type = shape_msgs.msg.SolidPrimitive.BOX
        cube.dimensions = [0.06,0.06,0.06]
        collision_object_cube.primitives = [cube]
        collision_object_cube.primitive_poses = [desired_pose]

        #append collision_object_table to the planning scene msg.
        plscene_msg.world.collision_objects.append(collision_object_cube)
        #plscene_msg.is_diff = True
        #and return it.
        return plscene_msg

def main():
    try:
        print "============ beginning the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
        tutorial = MoveGroupPythonIntefaceTutorial()
        plscene_msg = moveit_msgs.msg.PlanningScene()
        tutorial.planning_scene_publisher.publish(plscene_msg)

        #print "============ Press `Enter` to execute a movement using a pose goal ..."
        #raw_input()
        #tutorial.go_to_pose_goal()

        print "============ spawn cube and table ..."
        plscene_msg = tutorial.add_table(plscene_msg)
        #this pose msg will designate the desired position of each cubes generated.
        pose1 = geometry_msgs.msg.Pose()
        pose1.position.x =0.55
        pose1.position.y =-0.10
        pose1.position.z =0.13
        pose1.orientation.w = 1.0                            #pose(0.55 -0.1 0.13)
        plscene_msg = tutorial.add_cube(plscene_msg, pose1)  #cube_1
        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x =0.45
        pose2.position.y =-0.10
        pose2.position.z =0.13
        pose2.orientation.w = 1.0                            #pose(0.45 -0.1 0.13)
        plscene_msg = tutorial.add_cube(plscene_msg, pose2)  #cube_2
        pose3 = geometry_msgs.msg.Pose()
        pose3.position.x =0.55
        pose3.position.y =0.10
        pose3.position.z =0.13
        pose3.orientation.w = 1.0                            #pose(0.45 0.1 0.13)
        plscene_msg = tutorial.add_cube(plscene_msg, pose3)  #cube_3
        pose4 = geometry_msgs.msg.Pose()
        pose4.position.x =0.45
        pose4.position.y =0.10
        pose4.position.z =0.13
        pose4.orientation.w = 1.0                            #pose(0.55 0.1 0.13)
        plscene_msg = tutorial.add_cube(plscene_msg, pose4)  #cube_4

        red = std_msgs.msg.ColorRGBA()
        red.r = red.a = 1.0
        red.b = red.g = 0.0

        cyan = std_msgs.msg.ColorRGBA()
        cyan.r = 0.0
        cyan.b = cyan.a = cyan.g = 1.0

        blue = std_msgs.msg.ColorRGBA()
        blue.b = blue.a = 1.0
        blue.g = blue.r = 0.0

        white = std_msgs.msg.ColorRGBA()
        white.r = white.a = 1.0
        white.b = white.g = 1.0

        table_color = moveit_msgs.msg.ObjectColor()
        cube_1_color = moveit_msgs.msg.ObjectColor()
        cube_2_color = moveit_msgs.msg.ObjectColor()
        cube_3_color = moveit_msgs.msg.ObjectColor()
        cube_4_color = moveit_msgs.msg.ObjectColor()

        plscene_msg.object_colors.append(table_color)
        plscene_msg.object_colors.append(cube_1_color)
        plscene_msg.object_colors.append(cube_2_color)
        plscene_msg.object_colors.append(cube_3_color)
        plscene_msg.object_colors.append(cube_4_color)

        plscene_msg.object_colors[0].id = "table"
        plscene_msg.object_colors[0].color = white
        plscene_msg.object_colors[1].id = "cube_1"
        plscene_msg.object_colors[1].color = red
        plscene_msg.object_colors[2].id = "cube_2"
        plscene_msg.object_colors[2].color = blue
        plscene_msg.object_colors[3].id = "cube_3"
        plscene_msg.object_colors[3].color = cyan
        plscene_msg.object_colors[4].id = "cube_4"
        plscene_msg.object_colors[4].color = blue

        plscene_msg.is_diff = True
        tutorial.planning_scene_publisher.publish(plscene_msg)

        rospy.sleep(0.5)
        print "============ pick and place..."
        #from now on, pose designates position of eef.
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.45
        pose.position.y = -0.10
        pose.position.z = 0.28                             #pose(0.45 -0.1 0.28)
        pose.orientation.x =0.707
        pose.orientation.y =-0.707
        pose.orientation.z =0.0
        pose.orientation.w =0.0                             #rpy(0,-pi, -pi/2)

        tutorial.go_to_pose_goal(pose)

        rospy.sleep(0.5)

        grasping_group = 'robotiq'
        touch_links = tutorial.robot.get_link_names(group=grasping_group)
        tutorial.group.attach_object( 'cube_2', tutorial.eef_link, touch_links=touch_links)

        pose.position.x = 0.55
        pose.position.z = 0.34                             #pose(0.55 -0.1 0.34) rpy(0,-pi, -pi/2)
        tutorial.go_to_pose_goal(pose)
        tutorial.group.detach_object('cube_2')

        plscene_msg.object_colors[2].color = blue
        plscene_msg.world.collision_objects[2].primitive_poses[0].position.x = 0.55
        plscene_msg.world.collision_objects[2].primitive_poses[0].position.z = 0.19
        tutorial.planning_scene_publisher.publish(plscene_msg)

        rospy.sleep(0.5)

        pose.position.x = 0.45
        pose.position.y = 0.10
        pose.position.z = 0.28
        tutorial.go_to_pose_goal(pose)

        rospy.sleep(0.5)

        tutorial.group.attach_object( 'cube_4', tutorial.eef_link, touch_links=touch_links)
        pose.position.x = 0.55
        pose.position.z = 0.34
        tutorial.go_to_pose_goal(pose)
        tutorial.group.detach_object('cube_4')

        plscene_msg.object_colors[4].color = blue
        plscene_msg.world.collision_objects[4].primitive_poses[0].position.x = 0.55
        plscene_msg.world.collision_objects[4].primitive_poses[0].position.z = 0.19
        tutorial.planning_scene_publisher.publish(plscene_msg)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print "============ Python pick place demo complete!"
if __name__ == '__main__':
  main()

## END_TUTORIAL
