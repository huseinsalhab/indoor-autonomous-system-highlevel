#! /usr/bin/env python

import roslaunch
import rospy
from time import sleep 
from std_msgs.msg import String

def blah():
    package = 'ians_control'
    executable = 'ians_control_server'
    executable = 'keyboard_listener.py'
    executable = 'motor_controller.py'
    server_node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(server_node)
    print(process.is_alive)
    sleep(30)
    process.stop()


def master_callback(msg):
    print('Just heard', msg )
    print('Booting up motor node...')
    package = 'ians_control'
    executable = 'motor_controller.py'
    server_node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(server_node)
    print(process.is_alive)
    sleep(10)
    process.stop()

def main():
    # Create a ROS Listener to recieve future messages
    print('Booting up master listener...')
    rospy.init_node('master_listener')

    rospy.Subscriber('master_msgs', String, master_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
