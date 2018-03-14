# Control: Keyboard commands, Server commands, and Motor commands.
Manually teleop the robot. Also handles server communication

## Setup

To compile the non ROS file `server_interface.cpp`, use the command

```
g++ -o server_interface server_interface.cpp -l mosquitto
```
