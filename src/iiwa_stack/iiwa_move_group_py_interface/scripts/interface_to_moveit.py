#!/usr/bin/env python

"""
This script plays the role of client in picknmove control service for simulated kuka iiwa
"""
# ----------------- To be able to run script from terminal ------------------
import sys
import os
from os.path import expanduser
home_path = expanduser("~")
sys.path.append(home_path + '/iiwa_stack_ws')
# --------------------------------------------------------------------------

import csv
import os
import sys
import copy
import numpy as np
import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from iiwa_picknmove_control.srv import *


def main():        
    np.set_printoptions(precision=3)

    robot, scene, move_group, display_trajectory_publisher = init_moveit_interface()
    print_info(robot, move_group)    
    
    while not rospy.is_shutdown():
        ee_test_quat = np.array([None, None, None, None])
        # Option1: Get goal form user:
        print('Input ee x y z delimited with space:')
        str_ = str(input())
        ee_test_pos = np.array(str_.split()).astype(float)
        # --------------------------        

        # Option3: Get a random pose from MoveIt interface
        # random_pose = move_group.get_random_pose()
        # ee_test_pos = np.array([random_pose.pose.position.x, random_pose.pose.position.y, random_pose.pose.position.z])
        # ee_test_quat = np.array([random_pose.pose.orientation.x, random_pose.pose.orientation.y,
        #                          random_pose.pose.orientation.z, random_pose.pose.orientation.w])
        # print('Random move_group pose: {}'.format(random_pose))
        # --------------------------

        print('test_ee_pos: {}, test_ee_quat: {}'.format(ee_test_pos, ee_test_quat))
        state = move_group.get_current_pose()
        set_test_position_by_ee(move_group, ee_test_pos[0], ee_test_pos[1], ee_test_pos[2],
                                ee_test_quat[0], ee_test_quat[1], ee_test_quat[2], ee_test_quat[3])
        print('move_group.get_current_pose(): {}'.format(state))
        print('Current rpy: {}'.format(move_group.get_current_rpy()))

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()


def set_test_position_by_ee(move_group, x, y, z, qx=None, qy=None, qz=None, qw=None):
    pose_goal = geometry_msgs.msg.Pose()

    if qx is not None and qy is not None and qz is not None and qw is not None:
        # Set orientation as requested
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
    else:
        # Rotate around x axis 180 degrees
        pose_goal.orientation.x = 1
        pose_goal.orientation.y = 0.
        pose_goal.orientation.z = 0.
        pose_goal.orientation.w = 0.

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()


def set_home_position_by_joints(move_group):
    """Sets the robot joints initial configuration"""
    joint_home = move_group.get_current_joint_values()

    joint_home[0] = np.deg2rad(110)
    joint_home[1] = np.deg2rad(39)
    joint_home[2] = np.deg2rad(145)
    joint_home[3] = np.deg2rad(64)
    joint_home[4] = np.deg2rad(-161)
    joint_home[5] = np.deg2rad(79)
    joint_home[6] = np.deg2rad(-11)

    move_group.go(joint_home, wait=True)
    move_group.stop()


def init_moveit_interface():
    # Init a moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # RobotCommander provides info such as robot kinematic model and robot's cuurent joint states
    robot = moveit_commander.RobotCommander()

    # Remote interface for getting, setting, and updating the robot internal understanding of the surrounding world
    scene = moveit_commander.PlanningSceneInterface()

    # MoveGroupCommander is an interface to a planning group (group of joints).
    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    return robot, scene, move_group, display_trajectory_publisher


def print_info(robot, move_group):
    # Print the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print('1) Planning frame: %s' % planning_frame)

    # Print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print('2) End effector link: %s' % eef_link)

    # Print list of all the groups in the robot:
    group_names = robot.get_group_names()
    print('3) Available Planning Groups:', robot.get_group_names())

    joint_goal = move_group.get_current_joint_values()
    print('4) Printing robot current joint values')
    print(joint_goal)

    # Sometimes for debugging it is useful to print the entire state of the robot:
    state = robot.get_current_state()
    print('5) Printing robot state')
    print(state)
    print('(6) Pose reference frame: {}'.format(move_group.get_pose_reference_frame()))
    print('OK')


def create_robot_control_client():
    """Create client for robot control service"""

    # Wait until the service becomes available
    rospy.wait_for_service('picknmove_control')
    try:
        # Create a handle for calling the service
        control_robot = rospy.ServiceProxy('picknmove_control', MoveitInterface)
        return control_robot

    except rospy.ServiceException as e:
        print('Service handle creation failed: %s' % e)


def get_control_signal(service_handle, ku_ee_pos, ku_ee_vel, ku_obj_pos):
    print('ku_ee_vel: {}\nku_ee_pos: {}\nku_obj_pos: {}'.format(
        ku_ee_vel, ku_ee_pos, ku_obj_pos))
    try:
        # Call the service
        resp = service_handle(ku_ee_pos, ku_ee_vel, ku_obj_pos)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    main()
