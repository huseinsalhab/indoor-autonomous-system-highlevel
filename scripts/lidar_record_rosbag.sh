#!/bin/bash

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link scan 100 &
./src/ians_bagfiles/rosbag_run.sh

#rosnode kill `rosnode list | grep static_transform_publisher`
