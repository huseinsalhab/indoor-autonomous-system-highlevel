#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from motors import *  

motor = MotorController()

#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def callback(data):
    if data.data == "up" :
        motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 2.0)))
        rospy.loginfo(" up")
    elif data.data == "down":
        motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.0)))
        rospy.loginfo(" down")
    else:
        motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.0)))
        rospy.loginfo(" error")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    motor.pwm.set_pwm(motor.channel['A'], 0, int(calcTicks(motor.freq, 1.0)))
    listener()
