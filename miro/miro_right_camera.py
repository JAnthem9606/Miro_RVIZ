#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

rospy.init_node("Miro_Right_Camera")

def right_camera_callback(ros_image):
	bridge = CvBridge()
	cv_image = bridge.compressed_imgmsg_to_cv2(ros_image)
	cv2.imshow("Miro Right Camera",cv_image)
	cv2.waitKey(1)
	#print(cv_image.shape)
	
rospy.Subscriber("/miro/sensors/camr/compressed",CompressedImage,right_camera_callback,queue_size=100)
rospy.spin()
cv2.destroyAllWindows()
