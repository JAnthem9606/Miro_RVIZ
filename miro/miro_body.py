#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

rospy.init_node("Miro_Body")

def body_callback(data):
	head = data.data
	print("Body Sensor Value : " , head)

rospy.Subscriber("/miro/sensors/touch_body",UInt16,body_callback)
rospy.spin()
