#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("Miro_Light_Sensors")

def light_sensors_callback(data):
	lights = list(data.data)
	lights_labels = ['FRONT LEFT : ', 'RIGHT : ', 'REAR LEFT : ', 'RIGHT : ']
	print(lights_labels[0],round(lights[0],2),
	lights_labels[1],round(lights[1],2),
	lights_labels[2],round(lights[2],2),
	lights_labels[3],round(lights[3],2))
	

rospy.Subscriber("/miro/sensors/light",Float32MultiArray,light_sensors_callback)
rospy.spin()
