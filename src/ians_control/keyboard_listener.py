#! /usr/bin/env python

# File used to record keyboard events on a Raspberry Pi and 
# publish accordingly to a ROS topic

import sys
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import pygame

last_char = ''

class KeyboardListener(object):
    """Class representing the keyboard listener."""

    def __init__(self):
        """Initialize pygame"""
        pygame.init()
        pygame.display.set_caption(u'Keyboard events')
        pygame.display.set_mode((400, 400))

    def listen(self, callback):
        """Listen for events to happen and passes the pressed character
        + an indicator of key press vs key release into the callback function"""
        
        ## Get any new events before publishing to ROS
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                try:
                    last_char = chr(event.key)
                except:
                    pass
            elif event.type == pygame.KEYUP:
                last_char = ''
            elif event.type == pygame.QUIT:
                pygame.display.quit()
                pygame.quit()
                sys.exit()
        ## Publish to geo twist topic based on whatever key was last pressed
        msg = Twist()
        if last_char == 'w':
            # Drive forward
            msg.linear.x = 10
        if last_char == 's':
            # Drive backward
            msg.linear.x = -10
        if last_char == 'd':
            # Turn right
            msg.angular.z = -1
        if last_char == 's':
            # Turn left
            msg.angular.z = 1
        # #Publish the message after creating the struct
        callback(msg)    
                                                   
def keyboard_event_publisher():
    """Listens for keyboard events and publishes them to the specified topic"""
    key_capture = KeyboardListener()

    geo_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    rospy.init_node('keyboard_event_publisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
       key_capture.listen(geo_twist_pub.publish)
       rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_event_publisher()
    except rospy.ROSInterruptException:
        pass
