#!/usr/bin/env python
import requests
from subprocess import call

server_ip = 'http://35.229.88.91'
endpoint = '/v1/robot_receive_map'

png_name = '/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/map.png'
png_name = '/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/random_detail.jpg'
# png_name = 'random_detail.jpg'

def post_image(data):
    """ Finds the map .pgm file, converts to .png, and sends it to the specified
    server endpoint
    """
    # POST map image to server
    with open(png_name, 'rb') as f:
        r = requests.post(server_ip + endpoint, files={png_name: f})
        # r = requests.post('https://requestb.in/1ltofds1', files={png_name: f})

if __name__ == '__main__':
    post_image('blah')
