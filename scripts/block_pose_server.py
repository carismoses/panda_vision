#!/usr/bin/env python

import re
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from panda_vision.srv import GetBlockPoses, GetBlockPosesResponse
from panda_vision.msg import BlockPose, BlockCameraPose
from panda_vision.scripts import rotation_util

# NOTE(izzy): in order to be able to continuously collect frames from the
# camera and also handle server requests, I've decided to have a publisher
# node which constantly publishes block poses as they are observed, and
# the block_pose_server is a subscriber to that node

class BlockPoseServer:
    def __init__(self):
        rospy.init_node('block_pose_server')
        rospy.Service('get_block_poses', GetBlockPoses, self.handle_get_block_poses)
        print('Get block poses server ready.')
        
        # time to wait for pose estimates (sec)
        self.sleep_time = 5
        
        # poses will accumulate here
        self.block_poses = {}
        
    def handle_get_block_poses(self, req):    
        # get camera block pose topics
        all_topics = rospy.get_published_topics()
        camera_block_pose_topics = [topic for topic in all_topics if 'camera_block_pose' in topic]
        
        # subscribe to them
        for topic in camera_block_pose_topics:
            pose_sub = rospy.Subscriber(topic, BlockCameraPose, self.block_pose_callback)
            self.block_poses[topic] = []
        
        # spin to accumulate pose estimates
        rospy.sleep(self.spin_time)
        
        # average poses over time and group by block id
        camera_block_avg_poses = {}
        for topic, block_poses in self.block_poses.items():
            matches = re.match(r'(.*)/block_(.*)_camera_(.*)', topic)
            block_id = matches.group(2)
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
            pose.pose.position = Point(*trans)
            pose.pose.orientation = Quaternion(*quat)
            block_pose = BlockPose(block_id, pose)
            final_avg_poses.append(block_pose)
        
        # publish poses
        for block_pose in final_avg_poses:
            topic = 'avg_block_pose/'+block_pose.block_id
            pub = rospy.Publisher(topic, BlockPose, queue_size=10)
            pub.publish(block_pose)
        
        # flush out poses
        self.block_poses = {}
        
        return GetBlockPosesResponse(final_avg_poses)
        
    def block_pose_callback(self, data):
        topic = 'camera_block_pose/block_'+str(data.block_pose.block_id)+'_camera_'+str(data.camera_id)
        trans = np.array([data.block_pose.pose.position.x, 
                        data.block_pose.pose.position.y,
                        data.block_pose.pose.position.z])
        quat = np.array([data.block_pose.pose.orientation.x,
                        data.block_pose.pose.orientation.y,
                        data.block_pose.pose.orientation.z,
                        data.block_pose.pose.orientation.w])
        R = rotation_util.quat_to_rot(quat)
        T = rotation_util.Rt_to_pose_matrix(R, trans)
        self.block_poses[topic].append(T)

if __name__ == '__main__':
    bps = BlockPoseServer()
    rospy.spin()