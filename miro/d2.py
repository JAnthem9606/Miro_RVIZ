#!/usr/bin/env python
import rospy
import threading
from callbacks import sonar_callback, light_sensors_callback, mic_callback
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Int16MultiArray
import streamlit as st
from lit import streamlit_display

class SensorDataManager:
    def __init__(self):
        # Initialize any variables or objects
        pass

    def ros_node(self):
        """Initialize ROS node and subscribe to topics"""
        rospy.init_node("Miro_Sonar_data", disable_signals=True)
        rospy.Subscriber("/miro/sensors/sonar", Range, sonar_callback)
        rospy.Subscriber("/miro/sensors/light", Float32MultiArray, light_sensors_callback)
        rospy.Subscriber("/miro/sensors/mics", Int16MultiArray, mic_callback)
        rospy.spin()

    def start(self):
        """Start ROS and Streamlit interface in separate threads"""
        # Start the ROS node in a separate thread
        ros_thread = threading.Thread(target=self.ros_node)
        ros_thread.start()

        # Start Streamlit display update in the main thread
        streamlit_display()

if __name__ == '__main__':
    sensor_manager = SensorDataManager()
    sensor_manager.start()
