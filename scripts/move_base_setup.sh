#!/bin/bash

# This script assumes launch_everything.sh has already been run
rosservice call start_motor

# Using a hardcoded map name, change if needed!
rosrun map_server map_server \
/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/first-floor-test.yaml &

roslaunch ians_navigation navstack_standup.launch
