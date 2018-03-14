#!/bin/bash

# source /home/ubuntu/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

roslaunch ians_control ians_ctl.launch & # Spin up Motors and Keyboard

/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_control/server_interface.out

sleep 10 # Wait for the RPLidar node to fully boot before calling stop motor
rosservice call stop_motor # Stop LiDAR spinning
# roslaunch ians_project base.launch >> ~/log 2>> ~/errorLog
