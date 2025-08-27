#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

# Lift =>  1 (Down) 0 (Up)
# Yaw  =>  1 (Left) -1 (Right)

# Initialize the ROS node
rospy.init_node('kinematic_joints_publisher', anonymous=True)
print("Kinematic Joints of Miro-e can be controlled using following attribuites of the Joint State ")
print("1: Tilt")
print("2: Lift")
print("3: Yaw")
print("4: Pitch")

def miro_kinematics():
	vel = 0.1
	eff = 0.1
	# Create a publisher for the /miro/control/kinematic_joints topic
	
	tilt = float(input("Enter Tilt Position : "))
	lift = float(input("Enter Lift Position : "))
	yaw = float(input("Enter Yaw Position : "))
	pitch = float(input("Enter Pitch Position : "))
	
	#vel = float(input("Enter Pitch Position"))
	#eff = float(input("Enter Pitch Position"))
	
	pub = rospy.Publisher('/miro/control/kinematic_joints', JointState, queue_size=10, latch=True)

	# Create and fill the JointState message
	joint_state_msg = JointState()
	joint_state_msg.name = ['tilt', 'lift', 'yaw', 'pitch']
	joint_state_msg.position = [tilt, lift, yaw, pitch]
	joint_state_msg.velocity = 4 * [vel]
	joint_state_msg.effort = 4 * [eff]

	# Publish the message at 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(joint_state_msg)
		rate.sleep()
	rospy.cleanup()

miro_kinematics()
