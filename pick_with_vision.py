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
from aruco_vision.cal import dist, mtx
from aruco_vision.rotation_util import *

from pb_robot.panda import Panda

# create the aruco dictionary and default parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params =  aruco.DetectorParameters_create()

# 8 x 3 matrix of -1, 1 to compute the corners of the blocks (used in draw_block)
signed_corners = np.array([c for c in itertools.product([-1, 1], repeat=3)])

def get_block_poses_in_camera_frame(ids, corners, color_image=None):
    tag_id_to_block_pose = {}
    for i in range(0, ids.size):
        # pull out the info corresponding to this block
        tag_id = ids[i][0]
        block_id = tag_id // 6

        # detect the aruco tag
        marker_length = 5.4/100.0 # in meters
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(
            corners[i], marker_length, mtx, dist)
        # pose of the tag in the camera frame
        print('rvec:', rvec)
        print('rod:', cv2.Rodrigues(rvec))
        #input('Next?')
        X_CT = Rt_to_pose_matrix(cv2.Rodrigues(rvec)[0], tvec)
        # pose of the object in camera frame
        # X_TO = np.linalg.inv(marker_info["X_OT"])
        # X_CO = X_CT @ X_TO

    return X_CT

def draw_block(X_CO, color_image, draw_axis=True):
    """ Draw dots at the corners of the block at the specified camera-frame
    pose and with the given dimensions.

    The top corners are lighter in color and the bottom corners are darker.

    Arguments:
        X_CO {np.ndarray} -- 4x4 pose of the object in the camera frame
        dimensions {np.ndarray} -- x,y,z dimensions of the object
        color_image {np.ndarray} -- the image into which to draw the object

    Optional
        draw_axis {bool} -- Draw the block axis at the COG. Default True

    Side effects:
        Modifies color_image
    """
    if draw_axis:
        R_CO, t_CO = pose_matrix_to_Rt(X_CO)
        aruco.drawAxis(color_image, mtx, dist, R_CO, t_CO, 0.1)

def main(arm, gripper):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    #config.enable_stream(rs.stream.depth, 1920, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pb_robot.utils.connect(use_gui=False)
    robot = Panda()
    robot_pose = numpy.eye(4)
    robot.set_transform(robot_pose)

    try:
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            # depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # convert image to numpy array
            color_image = np.asarray(color_frame.get_data())
            # convert to grayscale
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            # detect aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                gray_image, aruco_dict, parameters=aruco_params)

            # In this code, only have one tag on an image.
            if ids is not None:
                # estimate the pose of the blocks (one for each visible tag)
                X_CO = \
                    get_block_poses_in_camera_frame(ids, corners)

                R_CO, T_CO = pose_matrix_to_Rt(X_CO)
                draw_block(X_CO, color_image)

                #transform = tfBuffer.lookup_transform('panda_link0', 'camera_color_optical_frame', rospy.Time())

                point = PoseStamped()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = "camera_color_optical_frame"
                point.pose.position.x = T_CO[0]
                point.pose.position.y = T_CO[1]
                point.pose.position.z = T_CO[2]

                q = rot_to_quat(R_CO)
                point.pose.orientation.x = q[0]
                point.pose.orientation.y = q[1]
                point.pose.orientation.z = q[2]
                point.pose.orientation.w = q[3]

                world = tfBuffer.transform(point, 'panda_link0')

                p_w = [world.pose.position.x,
                       world.pose.position.y,
                       world.pose.position.z]
                o_w = [world.pose.orientation.x,
                       world.pose.orientation.y,
                       world.pose.orientation.z,
                       world.pose.orientation.w]
                R_w = quat_to_rot(o_w)
                X_w = Rt_to_pose_matrix(R_w, p_w)
                print('T:', X_w)

                curr_q = arm.joint_angles()

                curr_tform = robot.arm.ComputeFK(arm.convertToList(curr_q))
                curr_tform[0:3, 3] = p_w
                curr_tform[2, 3] += 0.098 + 0.05

                joints = robot.arm.ComputeIK(curr_tform)
                print('goal:', curr_tform)
                print('q:', joints)

                x = input('Move?')
                if x == 'y':
                    arm.move_to_joint_positions(arm.convertToDict(joints))
                    input('Return to neutral?')
                    arm.move_to_neutral()



            cv2.imshow('Aruco Frames', color_image)
            cv2.waitKey(1)

    finally:
        # Stop streaming
        pipeline.stop()


if __name__ == '__main__':
    rospy.init_node('aruco_pick')
    arm = franka_interface.ArmInterface()
    gripper = franka_interface.GripperInterface()

    main(arm, gripper)
