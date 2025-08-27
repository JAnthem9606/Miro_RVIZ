#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('miro_wheel_cmd')

def encoders_callback(data):
	left_wheel , right_wheel = data.data
	left_wheel , right_wheel = round(left_wheel,2) , round(right_wheel,2)
	print(f'Left Wheel Velocity : {left_wheel} Right Wheel Velocity : {right_wheel}')

rospy.Subscriber("miro/sensors/wheel_speed_cmd",Float32MultiArray,encoders_callback)
rospy.spin()
