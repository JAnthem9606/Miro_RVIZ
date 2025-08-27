#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int64

# Initialize the node and publishers
rospy.init_node('miro_wheel_encoders')
lwheel = rospy.Publisher("/lwheel", Int64, queue_size=10)
rwheel = rospy.Publisher("/rwheel", Int64, queue_size=10)

rate = rospy.Rate(50)
# Initialize current encoder counts
current_left_count = 0
current_right_count = 0

def encoders_callback(data):
    global prev_left_wheel, prev_right_wheel, current_left_count, current_right_count
    
    # Extract current wheel speed data
    left_wheel, right_wheel = data.data
    left_wheel = round(left_wheel, 2)
    right_wheel = round(right_wheel, 2)

    # Update encoder counts based on the change
    if left_wheel > 0:
        current_left_count += 1
    elif left_wheel < 0:
        current_left_count -= 1

    if right_wheel > 0:
        current_right_count += 1
    elif right_wheel < 0:
        current_right_count -= 1

    # Update the previous values to the current ones
    #prev_left_wheel = left_wheel
    #prev_right_wheel = right_wheel

    # Publish the updated encoder counts
    lwheel.publish(current_left_count)  # Scale the count by 600 (as in your original code)
    rwheel.publish(current_right_count)
    rate.sleep()

    # Print for debugging
    print(f'Left Encoder: {current_left_count} Right Encoder: {current_right_count}')

# Subscribe to the wheel speed topic
rospy.Subscriber("miro/sensors/wheel_speed_opto", Float32MultiArray, encoders_callback)

# Keep the node running
rospy.spin()
