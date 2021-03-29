Note the data collection code runs with the `calibration_ws` workspace (python2).

To collect data, first change the roslaunch file:
`realsense-ros/realsense2_camera/launch/launch_cameras.launch`

Make sure the launch and throttle nodes are only uncommented for the desired camera. I wasn't able to get this working with multiple cameras yet.

Run the launch file:
`roslaunch realsense2_camera launch_cameras.launch`

If you want to publish custom intrinsics, run (for the appropriate camera):
`python intrinsics_A.py`

Finally rosbag the throttled topics and intrisics:
`rosbag record /camera_B/color/camera_info /camera_B/color/image_raw_throttled /camera_B/depth/image_rect_raw_throttled /camera_B/color/custom_cal  /camera_B/depth/camera_info
`

Compress the final rosbag for easy storage:
`rosbag compress <name>.bag`
