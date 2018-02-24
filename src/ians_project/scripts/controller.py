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

class KeyboardController(object):
    """Class representing the keyboard controller."""
       
    last_key_pressed = 0 

    def __init__(self):
        """Initialize the joystick components"""
        # print('pygame.init returned:', pygame.init())
        pygame.init()
        pygame.display.set_caption(u'Keyboard events')
        pygame.display.set_mode((400, 400))

        # pygame.event.set_allowed([pygame.KEYDOWN, pygame.KEYUP])

        # self.key_data = defaultdict(bool)

        # self.key_callback_fxns = defaultdict(list)

    def listen(self, f):
        """Listen for events to happen"""
        while True:

            for event in pygame.event.get():
                # print(event.type == pygame.JOYAXISMOTION)
                if event.type == pygame.KEYDOWN:
                    # print('Hi', event.key)
                    # self.key_data[event.key] = True
                    try:
                        real_char = chr(event.key)
                        pub_str = 'd' + real_char 
                        f(pub_str)
                    except:
                        pass
                elif event.type == pygame.KEYUP:
                    try:
                        real_char = chr(event.key)
                        pub_str = 'u' + real_char 
                        f(pub_str)
                    except:
                        pass
                elif event.type == pygame.QUIT:
                    pygame.display.quit()
                    pygame.quit()
                    sys.exit()

def car_keyboard_talker():
    key_capture = KeyboardController()

    pub = rospy.Publisher('car_command', String, queue_size = 10)
    rospy.init_node('car_keyboard_talker', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
       key_capture.listen(pub.publish)
       rate.sleep()

if __name__ == '__main__':
    try:
        car_keyboard_talker()
    except rospy.ROSInterruptException:
        pass

