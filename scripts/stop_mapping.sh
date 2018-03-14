#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

map_path="/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/"

rosservice call stop_motor # Stop LiDAR spinning

# Possibly rm map.pgm?
rm "$map_path"map.pgm
rosrun map_server map_saver -f "$map_path"map # automatically adds .pgm ?!

# convert the newly created .pgm file to be a .png for the web server
convert "$map_path"map.pgm "$map_path"map.png

# POST the .png file to the web server
python "$map_path"map-to-server.py

# TODO: Kill Hector Node
rosnode kill /hector_height_mapping
