#!/bin/bash

# this script launches the set_nav_goal ROS node to update the robot's
# destination. the x and y coordinates are expected to be passed into this 
# script in $1 and $2, respectively. 

# make sure the number of arguments is correct
if [ "$#" -lt 2 ]
then
    echo "error: not enough arguments"
    echo "usage: set_nav_goal x y"
    exit 1
fi

# source ROS setup files to make sure ROS environment is correct
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

rostopic pub --once move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: '$1', y: '$2'}, orientation: {x: 0, y: 0, z: 0, w: 1}}}' &
