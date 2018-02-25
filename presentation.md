# Slide 1 Kelvin

Slide Title: Laser Only mapping and Localization 
- RPLidar has arrived, but no encoders
- Prototype: daniel mounted the LIDAR
- After Juan + Daniel setup RPLidar driver wanted to map daniels house

Slide Title: Hector Mapping Laser Scan
- As discussed, we used GMapping in gazebo, but requires odometry
- Solution: User Hector Mapping ROS Package. Can create maps without odometry data
- Hector mapping uses scan matching algorithm to create a map. 
- Previous scan and current scan are related to give a built picture
- We mapped Daniels living room+Kitchen (Next slide)


Slide Title: Picture
- <Add picture of mcmillan> 

Slide Title: Next Iteration: Map out JB 301 hallway 
- Brought the car and PI prototype to room 301
- Brought up teleop and hector nodes
- Created map of 301 hallways.

Slide Title: Picture
- <Add picture of hallway>

Slide Title: Experiment: Localization Laser Only
- Played around with hector mapping output
- The map to odometry transform provided by hector mapping can be used for localization
- Basically using hector mapping's laser scan algorithm to provide odometric data 
- Given an initial position, we were able to localize within 301 hallway
- However, was not perfect and odometry drifted over time


