#!/usr/bin/env python
import requests
from subprocess import call

server_ip = 'http://35.229.88.91'
endpoint = '/v1/robot_receive_map'

png_name = '/home/ubuntu/indoor-autonomous-system-highlevel/src/ians_mapping/map.png'

def main():
    """ Finds the map .png and sends it to the specified
    server endpoint
    """
    # POST map image to server
    with open(png_name, 'rb') as f:
        r = requests.post(server_ip + endpoint, files={'map.png': f})
        print(r)

if __name__ == '__main__':
    main()
