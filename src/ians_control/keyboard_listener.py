#! /usr/bin/env python

# File used to record keyboard events on a Raspberry Pi and
# publish accordingly to a ROS topic

import sys
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import pygame
from collections import defaultdict


class KeyboardListener(object):
    """Class representing a keyboard listener"""

    def __init__(self):
        pygame.init()
        pygame.display.set_caption(u'Keyboard events')
        pygame.display.set_mode((400, 400))
        self.keys = defaultdict(bool)

    def check_key(self, key, direction):
        """Sets the dict based on what key is currently pressed
        Ignores any non-ASCII keys (e.g. arrow keys)"""
        try:
            char = chr(key)
        except:
            return
        self.keys[char] = direction

    def build_message():
        msg = Twist()
        if self.keys['w']: # Forward
            msg.linear.x = 5
        if self.keys['s']: # Back up
            msg.linear.x = -5
        if self.last_char == 'd': # Turn right
            msg.angular.z = -25
        if self.last_char == 'a': # Turn left
            msg.angular.z = 25
        return msg

    def listen(self, callback):
        """Listen for events to happen and passes the pressed character
        + an indicator of key press vs key release into the callback function"""

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                self.check_key(event.key, True)
            elif event.type == pygame.KEYUP:
                self.check_key(event.key, False)
            elif event.type == pygame.QUIT:
                pygame.display.quit()
                pygame.quit()
                sys.exit()

            msg = build_message()
            callback(msg)

def keyboard_event_publisher():
    """Listens for keyboard events and publishes them to the specified topic"""
    key_capture = KeyboardListener()

    geo_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    rospy.init_node('keyboard_event_publisher', anonymous=True)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
       key_capture.listen(geo_twist_pub.publish)
       rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_event_publisher()
    except rospy.ROSInterruptException:
        pass
