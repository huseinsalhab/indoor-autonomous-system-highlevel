#!/bin/bash

# source /home/ubuntu/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

# Wait for the RPLidar node to fully boot before calling stop motor

roslaunch ians_control ians_ctl.launch & # Spin up Motors, Keyboard, Server 

sleep 10
rosservice call stop_motor # Stop LiDAR spinning
# roslaunch ians_project base.launch >> ~/log 2>> ~/errorLog
