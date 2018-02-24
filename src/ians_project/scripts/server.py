#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Juan Huerta Feb 4
# this file pings the server if a car command happened
# python request library used to get contents of server
# docs.python-requests.org/en/master/

import sys
import rospy
from std_msgs.msg import String
import requests
from time import sleep

class ServerListener(object):
    """Class representing the keyboard controller."""
       
    last_key_pressed = 0 

    def __init__(self):
        """Initialize the joystick components"""

    def ping(self, f):
        """Listen for events to happen"""
        while True:
            try:
                current_request = requests.get('http://35.229.88.91/v1/command')
                if current_request.text == '0':
                    f('kill')            
            except:
                pass
            sleep(.5)
def car_server_talker():
    server = ServerListener()

    pub = rospy.Publisher('car_command', String, queue_size = 10)
    rospy.init_node('car_keyboard_talker', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        server.ping(pub.publish)
        rate.sleep()

if __name__ == '__main__':
    try:
        car_server_talker()
    except rospy.ROSInterruptException:
        pass

