#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from motors import * 
import drivetrain

steer = drivetrain.SteeringController()
carMotor = drivetrain.ThrottleController() 

# rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def callback(data):
    print (data)
    #print (data.linear)
    #turn on motor
    if data.data[1] == 'w' :

        if data.data[0] == 'd':
            carMotor.set_throttle(0.25)
        elif data.data[0] == 'u':
            carMotor.set_throttle(0)

        rospy.loginfo(' up')
    #turn off motor
    elif data.data[1] == 's':

        if data.data[0] == 'u':
            carMotor.set_throttle(0)
        elif data.data[0] == 'd':
            carMotor.set_throttle(-0.5)

        rospy.loginfo('down')
    #turn right
    elif data.data[1] == 'd':

        if data.data[0] == 'u':
            steer.turn(0)
        elif data.data[0] == 'd':
            steer.turn(1)

        rospy.loginfo('right')
    #turn left
    elif data.data[1] == 'a':

        if data.data[0] == 'u':
            steer.turn(0)
        elif data.data[0] == 'd':
            steer.turn(-1)
    
        rospy.loginfo('left')
    #kill
    elif data.data == 'kill':
        carMotor.set_throttle(0)
        steer.turn(0)
        
        rospy.loginfo('kill')
    else:
        #motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.0)))
        carMotor.set_throttle(0)
        rospy.loginfo(' error')
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('car_command', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    carMotor.set_throttle(0)
    listener()
