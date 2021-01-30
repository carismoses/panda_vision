import franka_interface
import rospy
import itertools
import pickle
import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import os
from std_msgs.msg import String
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PoseStamped
import pb_robot
import numpy
from rotation_util import *
from panda_vision.msg import NamedPose
from panda_vision.srv import GetBlockPoses
from pb_robot.panda import Panda


def move_to_blocks(arm, gripper, block_poses):
    block_id = 6 # options: 0, 2, 6
    move_to_pose = None
    for block_pose in block_poses:
        if block_pose.block_id == str(block_id):
            move_to_pose = block_pose.pose
    if not move_to_pose:
        print('Service did not return desired block pose')
        return

    pb_robot.utils.connect(use_gui=False)
    robot = Panda()
    robot_pose = numpy.eye(4)
    robot.set_transform(robot_pose)

    p_w = [move_to_pose.pose.position.x,
           move_to_pose.pose.position.y,
           move_to_pose.pose.position.z]
    o_w = [move_to_pose.pose.orientation.x,
           move_to_pose.pose.orientation.y,
           move_to_pose.pose.orientation.z,
           move_to_pose.pose.orientation.w]
    R_w = quat_to_rot(o_w)
    X_w = Rt_to_pose_matrix(R_w, p_w)
    print('T:', X_w)

    curr_q = arm.joint_angles()

    curr_tform = robot.arm.ComputeFK(arm.convertToList(curr_q))
    curr_tform[0:3, 3] = p_w
    curr_tform[2, 3] += 0.098 + 0.065 + 0.1 # 0.098

    approach = robot.arm.ComputeIK(curr_tform)

    curr_tform[2, 3] -= 0.1
    joints = robot.arm.ComputeIK(curr_tform, seed_q=approach)
    print('goal:', curr_tform)

    x = input('Move?')
    if x == 'y':
        arm.move_to_joint_positions(arm.convertToDict(approach))
        arm.move_to_joint_positions(arm.convertToDict(joints))
        input('Return to neutral?')
        arm.move_to_joint_positions(arm.convertToDict(approach))
        arm.move_to_neutral()

if __name__ == '__main__':
    rospy.init_node('aruco_pick')
    arm = franka_interface.ArmInterface()
    gripper = franka_interface.GripperInterface()

    rospy.wait_for_service('get_block_poses')
    try:
        get_block_poses = rospy.ServiceProxy('get_block_poses', GetBlockPoses)
        block_poses = get_block_poses()
        move_to_blocks(arm, gripper, block_poses.poses)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
