#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np
from geometry_msgs.msg import TwistStamped
import time as t

twsp = rospy.Publisher("/miro/control/cmd_vel",TwistStamped)
rospy.init_node("miro_move_on_sound")
rate = rospy.Rate(50)

def mic_callback(data):
	mic_data = data.data
	mic_data = np.array(mic_data)
	twist = TwistStamped()
	if max(mic_data) > 10000:
		print("Sound Detected, Moving")
		for i in range(0,15):
			twist.twist.linear.x = 1
			twsp.publish(twist)
			rate.sleep()
		
	else:
		print("No Sound, Stopping")
		for i in range(0,5):
			twist.twist.linear.x = 0
			twsp.publish(twist)
			rate.sleep()
		
rospy.Subscriber("/miro/sensors/mics",Int16MultiArray,mic_callback)
rospy.spin()
