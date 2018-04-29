#!/bin/bash
# Script that launches the Diff Drive package, keyboard listener, LiDAR node, and connects to Teensy

# source /home/ubuntu/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_control/server_interface.out &

# roslaunch ians_control ians_ctl.launch & # Spin up Motors and Keyboard
roslaunch ians_control ians_ctl_lowlevelPID.launch & # Spin up Motors and Keyboard

sleep 5

rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200 & # Connect to Teensy

sleep 7 # Wait for the RPLidar node to fully boot before calling stop motor
rosservice call stop_motor # Stop LiDAR spinning

