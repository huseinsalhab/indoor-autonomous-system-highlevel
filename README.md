# Indoor Autonomous Navigation System High Level

### Repository Hierarchy

The Indoor Autonomous System Software is contained within three repositories.

1. [indoor-autonomous-system-cloud](https://github.com/thedch/indoor-autonomous-system-cloud)
1. [indoor-autonomous-system-highlevel](https://github.com/thedch/indoor-autonomous-system-highlevel)
1. [indoor-autonomous-system-lowlevel](https://github.com/thedch/indoor-autonomous-system-lowlevel)

The cloud repository contains code to run the front facing Flask server on GCE. This presents the user with a control panel containing destinations inside the floor plan, and a kill switch.

The high level repository contains ROS code that runs on a Raspberry Pi. This deals with the ROS navigation stack, manual control of the robot, interfacing with the LiDAR, the cloud server, and the low level micro.

The low level repository contains C++ code that interfaces directly with the sensors and motors to control the direction of the robot and read wheel odometry and IMU data. This data is then passed back to the Raspberry Pi where it is processed by ROS.

### Dependencies
map_server: `sudo apt-get install ros-kinetic-map-server`

differential-drive package: `git clone https://github.com/jfstepha/differential-drive.git`

rplidar_ros: `sudo apt-get install ros-kinetic-rplidar-ros`

### Setting Up the Workspace
1. Clone this project.
1. `cd` into this project and `catkin_make`
1. `cd` into devel, and `source setup.bash`
1. `rosrun beginner_tutorials <module.py>`
1. Add `@/home/ubuntu/indoor-autonomous-system-highlevel/scripts/on_boot.sh` to  `~/.config/lxsession/Lubuntu/autostart`

### Coding Standards Doc

#### Golden Rule
Search on Google before asking for help.

#### Preamble
1. Every file, the first line should be your name, and date
2. Every file, the second line should be a short description purpose of the code written
3. Reserve 3-4 lines (multiline comment) for any links, resources, or help that you get while writing your code. The purpose is to keep tracks of links you use in case we run into similar problems again.
4. For step 3, do this for every file, or for blocks of code where the links are relevant.
5. Make comments short, but with good explanation. Nothing too long, nothing too short. Comments are for things that are not obvious, or to highlight and explain salient (relevant) lines of code.

#### Specifics
1. Use relevant names for variables. Unless it is an obvious abbreviation, or letter (ex. int "i" in a for loop, you don't need to name it "iterator".). Names will be snake case for local or global variables, but classes will be all capitalized camel case.
2. USE PARENTHESES. Do not rely on your knowledge of operator precedence. Use parentheses in complex logical structures (Logical, Boolean, Arithmetic operators).

		Example:
		This is BAD: (var1 && var2 || var3 ^ var4)
		This is GOOD: ( (var1 && var2) || (var 3 ^ var4) )

3. Everytime that you edit a file, please use SPACES, not TABS. Make sure UNIX line endings.


4. USE K&R braces and indentation style

		https://en.wikipedia.org/wiki/Indentation_style#K&R
		C/C++ Single line if statement should always have braces

5. Use single quotes for python strings.

#### Version Control

1. Make a branch for any features that you need to add. (Git checkout -b kelvin.new_feature)

2. The proper naming for a branch:

		first_name.feature_name
		kelvin.adding_slam_module

3. Git pull before you start writing code.
4. Write a sensible commit statement.
