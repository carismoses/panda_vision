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
from panda_vision.srv import GetBlockPosesWorld, GetBlockPosesWrist
from pb_robot.panda import Panda


def move_to_blocks(arm, gripper, block_poses):
    block_id = 6 # options: 0, 2, 6
    world_pose = None
    for block_pose in block_poses:
        if block_pose.block_id == str(block_id):
            world_pose = block_pose.pose
    if not world_pose:
        print('Service did not return desired block pose')
        return

    pb_robot.utils.connect(use_gui=False)
    robot = Panda()
    robot_pose = numpy.eye(4)
    robot.set_transform(robot_pose)


    # First move to where we think the block is in the global frame.
    p_w = [world_pose.pose.position.x,
           world_pose.pose.position.y,
           world_pose.pose.position.z]
    o_w = [world_pose.pose.orientation.x,
           world_pose.pose.orientation.y,
           world_pose.pose.orientation.z,
           world_pose.pose.orientation.w]
    R_w = quat_to_rot(o_w)
    X_w = Rt_to_pose_matrix(R_w, p_w)
    print('T:', X_w)

    # For now, keep orientation the same as it initially is.
    curr_q = arm.joint_angles()
    curr_tform = robot.arm.ComputeFK(arm.convertToList(curr_q))
    curr_tform[0:3, 3] = p_w
    curr_tform[2, 3] += 0.098 + 0.065 + 0.1 # 0.098

    approach_world = robot.arm.ComputeIK(curr_tform)

    x = input('Move?')
    if x == 'y':
        arm.move_to_joint_positions(arm.convertToDict(approach_world))

    # Then update the pose using the wrist frame.
    rospy.wait_for_service('get_block_poses_wrist')
    try:
        get_block_poses = rospy.ServiceProxy('get_block_poses_wrist', GetBlockPosesWrist)
        block_poses = get_block_poses()

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    wrist_pose = None
    for block_pose in block_poses.poses:
        if block_pose.block_id == str(block_id):
            wrist_pose = block_pose.pose
    if not wrist_pose:
        print('Wrist camera did not detect the desired pose.')
        return

    p_w = [wrist_pose.pose.position.x,
           wrist_pose.pose.position.y,
           wrist_pose.pose.position.z]
    o_w = [wrist_pose.pose.orientation.x,
           wrist_pose.pose.orientation.y,
           wrist_pose.pose.orientation.z,
           wrist_pose.pose.orientation.w]
    R_w = quat_to_rot(o_w)
    X_w = Rt_to_pose_matrix(R_w, p_w)
    print('T:', X_w)

    # For now, keep orientation the same as it initially is.
    curr_q = arm.joint_angles()
    curr_tform = robot.arm.ComputeFK(arm.convertToList(curr_q))
    curr_tform[0:3, 3] = p_w
    curr_tform[2, 3] += 0.098 + 0.065 + 0.1 # 0.098

    approach_wrist = robot.arm.ComputeIK(curr_tform, seed_q=approach_world)

    curr_tform[2, 3] -= 0.1
    grasp_wrist = robot.arm.ComputeIK(curr_tform, seed_q=approach_wrist)
    print('goal:', curr_tform)

    x = 'y'#input('Correct to wrist pose?')
    if x == 'y':
        arm.move_to_joint_positions(arm.convertToDict(approach_wrist))
        #input('Move to grasp.')
        arm.move_to_joint_positions(arm.convertToDict(grasp_wrist))
        arm.hand.grasp(0.02, 10, epsilon_inner=0.1, epsilon_outer=0.1)
        #input('Return to neutral?')
        arm.move_to_joint_positions(arm.convertToDict(approach_wrist))
        arm.move_to_neutral()
        arm.hand.open()

if __name__ == '__main__':
    rospy.init_node('aruco_pick')
    arm = franka_interface.ArmInterface()
    gripper = franka_interface.GripperInterface()

    rospy.wait_for_service('get_block_poses_world')
    try:
        get_block_poses = rospy.ServiceProxy('get_block_poses_world', GetBlockPosesWorld)
        block_poses = get_block_poses()
        move_to_blocks(arm, gripper, block_poses.poses)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
