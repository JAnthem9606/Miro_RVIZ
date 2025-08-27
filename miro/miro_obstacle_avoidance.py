#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

rospy.init_node("Miro_Sonar")
twsp = rospy.Publisher("/miro/control/cmd_vel",TwistStamped,queue_size=10)
mjs_pub = rospy.Publisher('/miro/control/kinematic_joints', JointState, queue_size=10, latch=True)
rate = rospy.Rate(50)

twist = TwistStamped()

def sonar_callback(data):
	distance = data.range
	threshold = 0.5
		
	if distance <= threshold:
		print(f'Obstacle Distance : {round(distance,2)} meter')
		print("There is an obstacle, avoiding")
		twist.twist.angular.z = 1
		twist.twist.linear.x = 0
		twsp.publish(twist)
		rate.sleep()
	
	else:
		print(f'Obstacle Distance : {round(distance,2)} meter')
		print("Clear, Moving")
		twist.twist.linear.x = 1
		twist.twist.angular.z = 0
		twsp.publish(twist)
		rate.sleep()

rospy.Subscriber("/miro/sensors/sonar",Range,sonar_callback)
rospy.spin() 
