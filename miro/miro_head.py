#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

rospy.init_node("Miro_Head")

def head_callback(data):
	head = data.data
	print("Head Sensor Value : " , head)

rospy.Subscriber("/miro/sensors/touch_head",UInt16,head_callback)
rospy.spin()
