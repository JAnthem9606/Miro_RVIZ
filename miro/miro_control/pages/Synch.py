#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
import time

# Global variables
latencies = {
    'wheel_speed_opto': [],
    'wheel_speed_back_emf': [],
    'body_vel': []
}
max_instances = 50

def measure_latency(topic_name, msg):
    end = time.time()  # Current time at callback
    now = rospy.get_time()  # ROS time when message was timestamped (approx.)
    latency_ms = (end - now) 
    latencies[topic_name].append(latency_ms)

    print(f"[{topic_name}] Latency: {latency_ms:.2f} ms")

    if len(latencies[topic_name]) >= max_instances:
        rospy.loginfo(f"\nFinal latencies for {topic_name} (ms):")
        print(latencies[topic_name])
        avg = sum(latencies[topic_name]) / len(latencies[topic_name])
        rospy.loginfo(f"Average Latency for {topic_name}: {avg:.2f} ms")

def callback_opto(msg):
    measure_latency('wheel_speed_opto', msg)

def callback_back_emf(msg):
    measure_latency('wheel_speed_back_emf', msg)

def callback_body_vel(msg):
    measure_latency('body_vel', msg)

def listener():
    rospy.init_node('latency_tester', anonymous=True,disable_signals=True)

    rospy.Subscriber("/sensors/wheel_speed_opto", Float32MultiArray, callback_opto)
    rospy.Subscriber("/sensors/wheel_speed_back_emf", Float32MultiArray, callback_back_emf)
    rospy.Subscriber("/sensors/body_vel", TwistStamped, callback_body_vel)

    rospy.loginfo("Latency measurement node started. Waiting for messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
