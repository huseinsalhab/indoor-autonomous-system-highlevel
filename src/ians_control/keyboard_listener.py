#! /usr/bin/env python

# File used to record keyboard events on a Raspberry Pi and 
# publish accordingly to a ROS topic

import sys
import rospy
from std_msgs.msg import Float32 
import pygame

class KeyboardListener(object):
    """Class representing the keyboard listener."""

    def __init__(self):
        """Initialize pygame"""
        pygame.init()
        pygame.display.set_caption(u'Keyboard events')
        pygame.display.set_mode((400, 400))

    def listen(self, r_callback, l_callback):
        """Listen for events to happen and passes the pressed character
        + an indicator of key press vs key release into the callback function"""
        fwd_speed = 75.0
        while True:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    try:
                        real_char = chr(event.key)
                        if real_char == 'w':
                            # Publish L + R fwd
                            r_callback(fwd_speed)
                            l_callback(fwd_speed)
                        if real_char == 's':
                            # Publish L + R back 
                            r_callback(fwd_speed * -1)
                            l_callback(fwd_speed * -1)
                        if real_char == 'd':
                            # Turn right
                            r_callback(fwd_speed * -1)
                            l_callback(fwd_speed)
                        if real_char == 'a':
                            # Turn left
                            r_callback(fwd_speed)
                            l_callback(fwd_speed * -1)
                        if real_char == 'x':
                            # Emergency brake 
                            r_callback(0)
                            l_callback(0)
                    except:
                        pass
                elif event.type == pygame.KEYUP:
                    try:
                        # Turn off all motors if keys are released
                        r_callback(0)
                        l_callback(0)
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

    right_pub = rospy.Publisher('rmotor', Float32, queue_size = 10)
    left_pub = rospy.Publisher('lmotor', Float32, queue_size = 10)
    rospy.init_node('keyboard_event_publisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
       key_capture.listen(right_pub.publish, left_pub.publish)
       rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_event_publisher()
    except rospy.ROSInterruptException:
        pass

