#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import sys
import rospy
from std_msgs.msg import String
import pygame
from collections import defaultdict

class KeyboardListener(object):
    """Class representing the keyboard listener."""

    def __init__(self):
        """Initialize the joystick components"""
        pygame.init()
        pygame.display.set_caption(u'Keyboard events')
        pygame.display.set_mode((400, 400))

    def listen(self, callback):
        """Listen for events to happen and passes the pressed character
        + an indicator of key press vs key release into the callback function"""
        while True:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    try:
                        real_char = chr(event.key)
                        pub_str = 'd' + real_char
                        callback(pub_str)
                    except:
                        pass

                elif event.type == pygame.KEYUP:
                    try:
                        real_char = chr(event.key)
                        pub_str = 'u' + real_char
                        callback(pub_str)
                    except:
                        pass

                elif event.type == pygame.QUIT:
                    pygame.display.quit()
                    pygame.quit()
                    sys.exit()

def keyboard_event_publisher():
    """ Listens for keyboard events and publishes them to the specified topic
    """
    key_capture = KeyboardListener()

    pub = rospy.Publisher('car_command', String, queue_size = 10)
    rospy.init_node('keyboard_event_publisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
       key_capture.listen(pub.publish)
       rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_event_publisher()
    except rospy.ROSInterruptException:
        pass

