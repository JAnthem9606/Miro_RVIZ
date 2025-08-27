#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

rospy.init_node("miro_mics")

def mic_callback(data):
	mic_data = data.data
	mic_data = np.array(mic_data)
	print(mic_data)
	
rospy.Subscriber("/miro/sensors/mics",Int16MultiArray,mic_callback)
rospy.spin()
