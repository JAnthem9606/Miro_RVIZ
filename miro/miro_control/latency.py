#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import csv
import os

# Path to store the log
LOG_FILE = os.path.expanduser("~/latency_log.csv")

def callback(msg):
    receive_time = rospy.Time.now()
    sent_time = msg.header.stamp

    # Skip latency calculation if header.stamp is zero (not set by publisher)
    if sent_time == rospy.Time():
        rospy.logwarn("Received message without valid header timestamp.")
        return

    # Calculate latency
    latency = (receive_time - sent_time).to_sec()
    rospy.loginfo(f"[Latency Test] Sent: {sent_time.to_sec():.6f} | Received: {receive_time.to_sec():.6f} | Latency: {latency:.6f} seconds")

    # Create log file with header if it doesn't exist
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Sent Time (s)", "Received Time (s)", "Latency (s)"])

    # Append latency data
    with open(LOG_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([sent_time.to_sec(), receive_time.to_sec(), latency])

def listener():
    rospy.init_node('latency_logger_continuous', anonymous=True)
    rospy.Subscriber("/miro/control/kinematic_joints", JointState, callback)
    rospy.loginfo("Latency Logger is running... Waiting for messages.")
    rospy.spin()

if __name__ == '__main__':
    listener()
