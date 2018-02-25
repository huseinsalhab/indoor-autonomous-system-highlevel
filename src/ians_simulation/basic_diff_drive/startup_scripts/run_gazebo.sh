#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
roslaunch diff_drive_gazebo diff_drive_world.launch
