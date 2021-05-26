#!/usr/bin/env python

"""
Publishes custom camera intrinsics to new topics
Copyright 2021 Massachusetts Institute of Technology
"""

import rospy
from sensor_msgs.msg import CameraInfo
from cal import get_custom_intrinsics

if __name__=="__main__":
    # Setup
    rospy.init_node("custom_intrinsics_publisher")
    camera_names = ["A", "B", "C"]
    rospy.loginfo(f"Publishing custom intrinsics for cameras: {camera_names}")

    # Initialize publishers for each camera
    pubs = {}
    for n in camera_names:
        topic_name = f"/camera_{n}/custom_intrinsics"
        pubs[n] = rospy.Publisher(topic_name, CameraInfo, queue_size=1)

    # Loop indefinitely and publish intrinsics for all cameras
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        for n in camera_names:
            K, dist = get_custom_intrinsics(n)
            info = CameraInfo()
            info.K = K.flatten().tolist()
            info.D = dist.squeeze().tolist()
            pubs[n].publish(info)
        r.sleep()
