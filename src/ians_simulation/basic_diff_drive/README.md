# Basic diff drive Simulation Package accumulation
Base simulation folder contains the packages needed to run the base simulation

## Differential Drive Prototype Simulation
This is the basic simulation of a 25x25 room.
The gazebo world instantiates with the room and the differential drive prototype of IANS

Note that changes in the diff_drive xacro will not reflect on gazebo simulation since the differential drive model was saved into the gazebo world.

If you need to change the differential drive model and have it reflect in this simulation, you need to change the launch file for the gazebo world setup.

Description:

Hokuyo LIDAR mesh + model, 6 Meter Radius, modeled as a 0.1m^3 box.

Red chassis, modeled as a 0.4m x 0.2m x 0.1m rectangular box.

Left and Right Wheels, modeled as 0.05m in radius (double check for accuracy)


## Simulation Bringup
Startup scripts are located in startup_scripts

