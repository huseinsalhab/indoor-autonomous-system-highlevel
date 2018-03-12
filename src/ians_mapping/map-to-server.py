#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import requests
from subprocess import call


server_ip = 'http://35.229.88.91'
endpoint = '/v1/robot_receive_map'

pgm_name = 'map.pgm'
png_name = 'map.png'

subscribe_topic = 'map_upload'

def callback(data):
    """ Finds the map .pgm file, converts to .png, and sends it to the specified
    server endpoint
    """
    if data.data == 'post':
        # Convert the pgm file to png file
        call(["convert", pgm_name, png_name])

        # POST map image to server
        with open(png_name, 'rb') as f:
            r = requests.post(server_ip + endpoint, files={filename: f})

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(subscribe_topic, String, callback)

    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
