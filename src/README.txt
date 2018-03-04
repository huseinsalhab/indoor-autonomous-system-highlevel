# Indoor Autonomous Navigation System 
## Source Tree

This directory  contains source code for teleoperation of car, as well as SLAM and Navigation stack

ians_control    -- contains nodes and launch files for teleop, server comms, embedded interface
ians_mapping    -- contains nodes for creating a map and SLAM
ians_navigation -- contains nodes for autonomous navigation

ians_scripts    -- contains extra scripts that might be needed

ians_simulation -- contains setup for Gazebo simulation
ians_bagfiles   -- contains sensor data recordings


├── ians_bagfiles
├── ians_control
│   └── launch
├── ians_mapping
│   ├── launch
│   ├── maps_baskin
│   ├── maps_mcmillan
│   └── rviz
├── ians_navigation
├── ians_scripts
├── ians_simulation
│   └── basic_diff_drive
│       ├── diff_drive_control
│       ├── diff_drive_description
│       ├── diff_drive_gazebo
│       ├── diff_drive_navigation
│       └── startup_scripts
25 directories
