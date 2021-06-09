#!/usr/bin/env python

import time
import numpy as np
import pyquaternion
import rospy
from panda_vision.srv import GetBlockPosesWorld


if __name__ == "__main__":
    rospy.init_node('aruco_example_client')
    while True:
        print("Requesting")
        rospy.wait_for_service('get_block_poses_world')
        get_block_poses = rospy.ServiceProxy('get_block_poses_world', GetBlockPosesWorld)
        try:
            poses = get_block_poses().poses
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # Calculate the difference between normal orientation and the estimated one.
        for named_pose in poses:
            print('-----')
            print('Block ID:', named_pose.block_id)
            pose = named_pose.pose.pose
            obs = pyquaternion.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            des = pyquaternion.Quaternion(1, 0, 0, 0)

            qd = des.inverse*obs
            print('Angle:', np.rad2deg(qd.angle))

        time.sleep(1)
