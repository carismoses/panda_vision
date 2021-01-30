import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np

block_0_pose = {'position': np.array([0.162, 0.463, 0.0725]), 'orientation': np.array([0.354, 0.935, 0.008, 0.007])}
block_2_pose = {'position': np.array([0.404, -0.222,  0.035]), 'orientation': np.array([0.948, -0.320, 0.010, 0.003])}
block_6_pose = {'position': np.array([0.688, 0.424, 0.065]), 'orientation': np.array([0.651, 0.759, 0.019, 0.010])}
poses = [block_0_pose, block_2_pose, block_6_pose]

def main():
    # setup the ros node
    pub_pose0 = rospy.Publisher('pose_0', PoseStamped, queue_size=10)
    pub_pose2 = rospy.Publisher('pose_2', PoseStamped, queue_size=10)
    pub_pose6 = rospy.Publisher('pose_6', PoseStamped, queue_size=10)
    pose_pubs = [pub_pose0, pub_pose2, pub_pose6]

    rospy.init_node("pose_pub")

    poses_to_publish = []
    for pose, pose_pub in zip(poses, pose_pubs):
        # create a pose message
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'base'

        # populate with the pose information
        p.pose.position = Point(*pose['position'])
        p.pose.orientation = Quaternion(*pose['orientation'])
        poses_to_publish.append(p)

    while not rospy.is_shutdown():
        for pose_pub, pose_to_pub in zip(pose_pubs, poses_to_publish):
            pose_pub.publish(pose_to_pub)


if __name__ == '__main__':
    main()
