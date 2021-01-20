# panda_vision

To use:
1. git clone this package into ```your_catkin_workspace/src```
2. build the package to ensure the message and server files are generated 

   ```catkin_make```
3. ```rosrun panda_vision get_block_poses_server.py```

Troubleshooting:
1. If you get errors, make sure that the file is executable by running: ```chmod +x scripts/get_block_poses_server.py```
2. This code uses python3 so make sure that you are set up to run ROS with python3. The first time you build your catkin_ws, 
use the command ```catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3```. After that you should be able to just ```catkin_make```
