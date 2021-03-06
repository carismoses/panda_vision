#!/usr/bin/env python

import re
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from panda_vision.srv import GetBlockPosesWorld, GetBlockPosesWrist, GetBlockPosesWorldResponse, GetBlockPosesWristResponse
from panda_vision.msg import BlockPose, BlockCameraPose
import rotation_util

# NOTE(izzy): in order to be able to continuously collect frames from the
# camera and also handle server requests, I've decided to have a publisher
# node which constantly publishes block poses as they are observed, and
# the block_pose_server is a subscriber to that node

class BlockPoseServer:
    def __init__(self, world_camera_names, wrist_camera_names):
        rospy.init_node('block_pose_server')

        # initialize tf buffer
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # advertize server
        rospy.Service('get_block_poses_world', GetBlockPosesWorld, self.handle_get_block_poses_world)
        rospy.Service('get_block_poses_wrist', GetBlockPosesWrist, self.handle_get_block_poses_wrist)


        print('Get block poses server ready.')

        # Keep track of which cameras are where.
        self.world_camera_names = world_camera_names
        self.wrist_camera_names = wrist_camera_names

        # poses will accumulate here
        self.block_poses = {}

    def handle_get_block_poses_world(self, req):
        return GetBlockPosesWorldResponse(self.get_poses('world', 3))

    def handle_get_block_poses_wrist(self, req):
        return GetBlockPosesWristResponse(self.get_poses('wrist', 1))

    def handle_get_grasp_pose(self,  req):
        return GetGraspPoseResponse(self.get_grasp_pose())

    def get_poses(self, camera_type, sleep_time):
        print('Processing service request...')
        # get camera block pose topics
        all_topics_and_types = rospy.get_published_topics()
        all_topics = [topic for topic, type in all_topics_and_types]

        if camera_type == 'wrist':
            valid_cameras = self.wrist_camera_names
        else:
            valid_cameras = self.world_camera_names
        camera_block_pose_topics = [topic for topic in all_topics if ('camera_block_pose' in topic and topic[-1] in valid_cameras)]

        # subscribe to them
        pose_subs = []
        self.block_poses = {}
        for topic in camera_block_pose_topics:
            pose_sub = rospy.Subscriber(topic, BlockCameraPose, self.block_pose_callback)
            pose_subs.append(pose_sub)
            self.block_poses[topic] = []

        # sleep to accumulate pose estimates
        rospy.sleep(sleep_time)

        # average poses over time and group by block id
        camera_block_avg_poses = {}
        print(self.block_poses.keys())
        for topic, block_poses in self.block_poses.items():
            matches = re.match(r'(.*)/block_(.*)_camera_(.*)', topic)
            block_id = matches.group(2)
            # Check if the block is no longer visible. Important for wrist camera.
            if len(block_poses) == 0:
                continue
            avg_pose = rotation_util.mean_pose(block_poses)

            if block_id in camera_block_avg_poses:
                camera_block_avg_poses[block_id].append(avg_pose)
            else:
                camera_block_avg_poses[block_id] = [avg_pose]

        # average poses over cameras
        final_avg_poses = []
        for block_id, block_poses in camera_block_avg_poses.items():
            avg_T = rotation_util.mean_pose(block_poses)
            R, trans = rotation_util.pose_matrix_to_Rt(avg_T)
            quat = rotation_util.rot_to_quat(R)
            pose = PoseStamped()
            pose.header.frame_id = 'base'
            pose.pose.position = Point(*trans)
            pose.pose.orientation = Quaternion(*quat)
            block_pose = BlockPose(block_id, pose)
            final_avg_poses.append(block_pose)

        # publish poses
        for block_pose in final_avg_poses:
            topic = 'avg_block_pose/block_'+block_pose.block_id
            pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
            pub.publish(block_pose.pose)

        # flush out poses and unsubscribe to topics
        [pose_sub.unregister() for pose_sub in pose_subs]
        self.block_poses = {}
        print('Server done estimating poses')
        return final_avg_poses

    def get_grasp_pose(self):
        """ Estimate the pose of the block currently held by the gripper
        """
        block_id = 0
        pose = PoseStamped()
        block_pose = BlockPose(block_id, pose)
        return block_pose

    def block_pose_callback(self, data):
        topic = '/camera_block_pose/block_'+str(data.block_pose.block_id)+'_camera_'+str(data.camera_id)
        pose_stamped_camera_frame = data.block_pose.pose

        # convert pose from camera frame to base frame (so can later average)
        transform = self.tf_buffer.lookup_transform('base',
                                pose_stamped_camera_frame.header.frame_id,
                                rospy.Time(0),          # get first available tf
                                rospy.Duration(0.5))    # wait for .5 sec
        pose_stamped_base_frame = tf2_geometry_msgs.do_transform_pose(pose_stamped_camera_frame, transform)

        trans = np.array([pose_stamped_base_frame.pose.position.x,
                        pose_stamped_base_frame.pose.position.y,
                        pose_stamped_base_frame.pose.position.z])
        quat = np.array([pose_stamped_base_frame.pose.orientation.x,
                        pose_stamped_base_frame.pose.orientation.y,
                        pose_stamped_base_frame.pose.orientation.z,
                        pose_stamped_base_frame.pose.orientation.w])
        R = rotation_util.quat_to_rot(quat)
        T = rotation_util.Rt_to_pose_matrix(R, trans)
        self.block_poses[topic].append(T)

if __name__ == '__main__':
    bps = BlockPoseServer(world_camera_names=['A', 'B'],
                          wrist_camera_names=['C'])
    rospy.spin()
