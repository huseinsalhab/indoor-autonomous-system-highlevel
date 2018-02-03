import time
import pygame
from threading import Thread
from controller import KeyboardController
import sys
sys.path.append('../')
from time import sleep


keyboard = KeyboardController()
# ps4 = PS4Controller()

def main():

    

    print('Entering the keyboard test module')
    thread = Thread(target=keyboard.listen)
    thread.start()
    while True:
        sleep(10)


if __name__ == '__main__':
    main()
