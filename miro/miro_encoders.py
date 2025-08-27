#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int64,Float32

rospy.init_node('miro_wheel_encoders')
lwheel = rospy.Publisher("/lwheel",Int64,queue_size=10)
rwheel = rospy.Publisher("/rwheel",Int64,queue_size=10)

def encoders_callback(data):
	left_wheel , right_wheel = data.data
	left_wheel , right_wheel = round(left_wheel,2) , round(right_wheel,2)
	print(f'Left Encoder : {left_wheel} Right Encoder : {right_wheel}')
	lwheel.publish(int(left_wheel*600))
	rwheel.publish(int(right_wheel*600))

rospy.Subscriber("miro/sensors/wheel_speed_opto",Float32MultiArray,encoders_callback)
rospy.spin()
