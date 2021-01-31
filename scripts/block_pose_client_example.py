#!/usr/bin/env python

import time

import rospy
from panda_vision.srv import GetBlockPosesWorld, GetBlockPosesWrist


if __name__ == "__main__":
    rospy.init_node('aruco_example_client')
    while True:
        print("Requesting")
        rospy.wait_for_service('get_block_poses_world')
        rospy.wait_for_service('get_block_poses_wrist')
        try:
            get_block_poses = rospy.ServiceProxy('get_block_poses_wrist', GetBlockPosesWrist)
            print(get_block_poses())

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        time.sleep(1)
