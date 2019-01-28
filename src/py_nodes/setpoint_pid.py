#!/usr/bin/env python
import rospy
import math
from math import *
from std_msgs.msg import Float64


def talker():

	setpoint_pub = rospy.Publisher('/setpoint',Float64, queue_size=10, latch=True)
	rospy.init_node('talker', anonymous=True)
	while not rospy.is_shutdown():
	    set_point = 0
	    setpoint_pub.publish(set_point)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        passs
