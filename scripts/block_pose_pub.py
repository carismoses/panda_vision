#!/usr/bin/env python
""" Massachusetts Institute of Technology

Izzybrand, 2020
"""

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import pyrealsense2 as rs

from panda_vision.msg import BlockPose, BlockCameraPose
from block_pose_est import BlockPoseEst
from rotation_util import *
from cal import get_custom_intrinsics

# map from camera serial no to camera ID
camera_lookup = {'032622074588':'A', '028522072401':'B', '032622073024': 'C'}

class BlockPosePublisher:

    def __init__(self):
        # setup the ros node
        rospy.init_node("block_pose_pub")

        # get active cameras to start publishing for
        print('Listing available realsense devices...')
        self.active_serial_numbers = []
        for i, device in enumerate(rs.context().devices):
            serial_number = device.get_info(rs.camera_info.serial_number)
            self.active_serial_numbers.append(serial_number)
            print(f'{i+1}. {serial_number}')

        # these dictionaries will hold all rospy Publishers where the keys are a string
        # camera_block_pose/camera_ID_block_ID_ and block_pose/block_ID_camera_ID
        # the topics they publish to will be the same name
        self.pose_publishers = {}
        self.camerapose_publishers = {}

        self.bpes = []
        for serial_number in self.active_serial_numbers:
            min_tags = 1
            if camera_lookup[serial_number] == 'C':
                min_tags = 1
            bpe = BlockPoseEst(self.publish_callback,
                               serial_number=serial_number,
                               intrinsics=get_custom_intrinsics(camera_lookup[serial_number]),
                               min_tags=min_tags)
            self.bpes.append(bpe)

    def publish_callback(self, block_id, camera_serial_no, X_CO):
        camera_id = camera_lookup[camera_serial_no]
        suffix = 'block_'+ str(block_id) + '_camera_' + str(camera_id)
        camera_pose_topic = 'camera_block_pose/' + suffix
        pose_topic = 'block_pose/' + suffix

        if camera_pose_topic not in self.camerapose_publishers:
            camerapose_pub = rospy.Publisher(camera_pose_topic, BlockCameraPose, queue_size=10)
            pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
            self.camerapose_publishers[camera_pose_topic] = camerapose_pub
            self.pose_publishers[pose_topic] = pose_pub
        else:
            camerapose_pub = self.camerapose_publishers[camera_pose_topic]
            pose_pub = self.pose_publishers[pose_topic]

        # create a pose message
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'camera_color_optical_frame_'+camera_id

        # populate with the pose information
        R_CO, t_CO = pose_matrix_to_Rt(X_CO)
        p.pose.position = Point(*t_CO)
        p.pose.orientation = Quaternion(*rot_to_quat(R_CO))

        # and publish the camera pose
        block_pose = BlockPose(str(block_id), p)
        camerapose_pub.publish(BlockCameraPose(camera_id, block_pose))
        pose_pub.publish(p)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            for bpe in self.bpes:
                bpe.step()
            rate.sleep()

        for bpe in self.bpes:
            bpe.close()



if __name__ == '__main__':
    bpp = BlockPosePublisher()
    bpp.run()
