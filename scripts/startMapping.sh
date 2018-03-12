#! /bin/bash

# this script launches the ROS nodes used for mapping the environment

# source files to make sure ROS environment is running correctly
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

# launch lidar node
roslaunch rplidar_ros rplidar.launch

# start publishing the static TF transform
static_transform_publisher 0 0 0 0 0 0 base_link scan 100

# start HECTOR, the SLAM package
roslaunch ians_project view_slam.launch 

