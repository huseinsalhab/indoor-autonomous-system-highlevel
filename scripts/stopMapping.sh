#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

rosrun map_server map_saver -f /home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/map.pgm

#update this to be a non-ROS node
rosrun ians_mapping map-to-server.py
rostopic pub /map_upload std_msgs/String "data: 'post'"

rosservice call stop_motor


