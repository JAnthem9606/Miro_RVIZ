#!/usr/bin/env python
import rospy
import threading
import time
import numpy as np
import streamlit as st
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32
from sensor_msgs.msg import Range

# Import the callback functions from the callbacks.py script
from callbacks import mic_callback, sonar_callback, light_sensors_callback

st.title("Miro Sensors Data")

s_col1, s_col2 = st.columns(2, gap="small")
s_col1.caption("Sonar Distance : ")

l_col1, l_col2 = st.columns(2, gap="small")
l_col1.caption("Light Intensities : ")

m_col1, m_col2 = st.columns(2, gap="small")
m_col1.caption("Mics Data : ")

light_holder = l_col2.empty()
sonar_holder = s_col2.empty()
mic_holder = m_col2.empty()

# Global variable to hold the latest sensor data
latest_distance = None
latest_light = None
latest_mic = None

# ROS Node function
def ros_node():
    rospy.init_node("Miro_Sonar_data", disable_signals=True)
    rospy.Subscriber("/miro/sensors/sonar", Range, sonar_callback)
    rospy.Subscriber("/miro/sensors/light", Float32MultiArray, light_sensors_callback)
    rospy.Subscriber("/miro/sensors/mics", Int16MultiArray, mic_callback)
    rospy.spin()

# Function to update streamlit display
def streamlit_display():
    while True:
        if latest_distance is not None:
            print(f"Updating Streamlit - Sonar: {latest_distance}, Light: {latest_light}, Mic: {latest_mic}")
            sonar_holder.write(latest_distance)
            light_holder.write(latest_light)
            mic_holder.write(latest_mic)
        time.sleep(0.02)  # 50 Hz

# Main function to run both ROS and Streamlit
def main():
    # Start ROS node in a separate thread
    ros_thread = threading.Thread(target=ros_node)
    ros_thread.start()

    # Start streamlit display update in the main thread
    streamlit_display()

if __name__ == '__main__':
    main()
