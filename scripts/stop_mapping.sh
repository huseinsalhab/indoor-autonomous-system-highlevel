#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/indoor-autonomous-system-highlevel/devel/setup.bash

map_path="/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/"

# rosrun map_server map_saver -f /home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/map.pgm

# convert the newly created .pgm file to be a .png for the web server
convert "$map_path"map.pgm "$map_path"map.png

# POST the .png file to the web server
curl \
-F "filecomment=This is an image file" \
-F "image=@""$map_path""map.png" \
https://requestb.in/1gyws5q1

# rosrun ians_mapping map-to-server.py
# rostopic pub /map_upload std_msgs/String "data: 'post'"

# rosservice call stop_motor # Stop LiDAR spinning


