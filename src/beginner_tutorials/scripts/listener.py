#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from motors import *  
import drivetrain

steer = drivetrain.SteeringController()
carMotor = drivetrain.ThrottleController() 
#motor = MotorController()

# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callback(data):
    #print (data)
    #print (data.linear)
    #turn on motor
    if data.linear.x == 2.0 :
        #motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.5)))
        carMotor.set_throttle(0.5)
        rospy.loginfo(" up")
    #turn off motor
    elif data.linear.x == -2.0:
        #motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.0)))
        carMotor.set_throttle(0)
        rospy.loginfo(" down")
    #turn right
    elif data.angular.z == -2.0:
        steer.turn(1)
        rospy.loginfo("right")
    #turn left
    elif data.angular.z == 2.0:
        steer.turn(-1)
        rospy.loginfo("left")
    else:
        #motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.0)))
        carMotor.set_throttle(0)
        rospy.loginfo(" error")
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    carMotor.set_throttle(0)
    listener()
