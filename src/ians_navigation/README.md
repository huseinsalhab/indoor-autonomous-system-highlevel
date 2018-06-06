# Navigation Files

Includes launch files for navigation stack, amcl, as well as move_base config

- Launch file contains static transform publishers to relate sensor readings to robotic base. Also contains Localization standup (AMCL) and Navigation standup (Move_Base)

- Additional documentation can be found at http://wiki.ros.org/navigation

## Move Base Config
Configuration files written in yaml, are referenced in the roslaunch launch file with rosparam.

### Base Local Planner Params
- max_vel_x - Maximum velocity expected for forward movement. (M/S)
- min_vel_x - Minimum velocity expected for forward movement. (To Avoid stall)
- max_vel_theta - Max velocity expected angular
- min_in_place_vel_theta - Minimum velocity angular

- escape_vel - Velocity for robotic movement during recovery movement
- acc_lim_ - Acceleration limits for robotic movement

- holonomic_robot - Specifies motion model (False = differential drive)
- vtheta_samples - Number of samples to take for velocity simulations of angular movement (Base planner tries movements before executing)
- vx_samples - Number of samples to take for velocity simulations of forward movement

### Costmap Common Params

- obstacle_range - Range at which to consider something an obstacle on costmap
- raytrace_range - Range for Raytracing
- robot_radius - Specifies the radius of the robot from the point of view of laser scan sensor. (For obstacle avoidance)
- inflation_radius - Specifies how much to inflate and obstacle on the costmap. (Tolerance to obstacle)
- observation_sources - Parameter for observation source

- laser_scan_sensor - Parameter specifying ROS LiDAR configuration , contains frame, data type, topic, etc


### Global / Local costmap params

- global_frame - Specifies the frame for costmap
- robot_base_frame - Specifies the TF frame for robot base
- update_frequency - Frequency to update Global or Local costmap
- publish_frequency - Frequency to publish tf transform
- resolution - Costmap resolution
- static_map - Whether costmap will be updated or not
- width / height in meters
