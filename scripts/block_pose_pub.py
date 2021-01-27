#!/usr/bin/env python
""" Massachusetts Institute of Technology

Izzybrand, 2020
"""

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import pyrealsense2 as rs

from panda_vision.msg import NamedPose
from block_pose_est import BlockPoseEst
from rotation_util import *

# map from camera serial no to camera ID
camera_lookup = {032622074588:'A', 028522072401:'B'}

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
        # camera_ID_block_ID_namedpose and block_ID_camera_ID_pose 
        # the topics they publish to will be of the same name
        self.pose_publishers = {}
        self.namedpose_publishers = {}

        bpes = []
        for serial_number in self.active_serial_numbers:
            bpe = BlockPoseEst(self.publish_callback, serial_number=serial_number)
            bpes.append(bpe)

        #pub_named_pose = rospy.Publisher('block_named_pose', NamedPose, queue_size=10)
        #pub_pose = rospy.Publisher('block_pose', PoseStamped, queue_size=10)

    def publish_callback(self, block_id, camera_serial_no, X_CO):
        prefix = str(block_id) + '-' + str(camera_serial_no)
        named_pose_topic = prefix + '_namedpose'
        pose_topic = prefix + '_pose'
        
        if named_pose_topic not in self.namedpose_publishers:
            namedpose_pub = rospy.Publisher(named_pose_topic, NamedPose, queue_size=10)
            pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
            self.namedpose_publishers[named_pose_topic] = namedpose_pub
            self.pose_publishers[pose_topic] = pose_pub
        else:
            namedpose_pub = self.namedpose_publishers[named_pose_topic]
            pose_pub = self.pose_publishers[pose_topic]
            
        # create a pose message
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'base'

        # populate with the pose information
        R_CO, t_CO = pose_matrix_to_Rt(X_CO)
        p.pose.position = Point(*t_CO)
        p.pose.orientation = Quaternion(*rot_to_quat(R_CO))

        # and publish the named pose
        serial_id = camera_lookup[camera_serial_no]
        pub_named_pose.publish(NamedPose(str(block_id), serial_id, p))
        pub_pose.publish(p)

    def run(self):
        while not rospy.is_shutdown():
            for bpe in bpes:
                bpe.step()

        for bpe in bpes:
            bpe.close()



if __name__ == '__main__':
    bpp = BlockPosePublisher()
    bpp.run()
