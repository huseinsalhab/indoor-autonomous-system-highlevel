#!/bin/bash

# this script launches the set_nav_goal ROS node to update the robot's
# destination. the x and y coordinates are expected to be passed into this 
# script in $1 and $2, respectively. 

# make sure the number of arguments is correct
if [ "$#" -lt 3 ]
then
    echo "error: not enough arguments"
    echo "usage: set_nav_goal x y facing"
    exit 1
fi

# source ROS setup files to make sure ROS environment is correct
source /opt/ros/kinetic/setup.bash
source /home/kyle/ians/indoor-autonomous-system-highlevel/devel/setup.bash

#rosrun ians_control set_nav_goal $1 $2

rostopic pub --once /initialpose geometry_msgs/PoseWithConvarianceStamped '{header: {stamp: now, frame_id: "map"}, pose: {pose: {position: {x: '$1', y: '$2'}, orientation: {x: 0, y: 0, z: '$3', w: 1}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}'
