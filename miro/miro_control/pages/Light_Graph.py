import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

# Streamlit UI

st.set_page_config(layout="centered")

st.title("Live Light Sensors Data Plot")
chart_placeholder = st.empty()

# Initialize an empty DataFrame
data = pd.DataFrame(columns=['Time', 'Front Left', 'Front Right', 'Rear Left', 'Rear Right'])

# Global variable to store light sensor values
light_values = [0.0, 0.0, 0.0, 0.0]

# ROS Callback function
def light_sensors_callback(msg):
    global light_values
    light_values = list(msg.data)  # Extract sensor values

try:
    rospy.init_node("light_sensor_streamlit_plot", anonymous=True,disable_signals=True)
except:
    print("No Initilization")

# Subscribe to ROS topic
rospy.Subscriber("/miro/sensors/light", Float32MultiArray, light_sensors_callback)

# Start time
start_time = time.time()

# Live update loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time
    
    # Append new data
    new_row = pd.DataFrame({
        "Time": [current_time],
        "Front Left": [light_values[0]],
        "Front Right": [light_values[1]],
        "Rear Left": [light_values[2]],
        "Rear Right": [light_values[3]]
    })
    
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot data
    fig, ax = plt.subplots()
    ax.plot(data["Time"], data["Front Left"], label="Front Left", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Front Right"], label="Front Right", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Rear Left"], label="Rear Left", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Rear Right"], label="Rear Right", marker="o",markersize=2.5)
    
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Light Intensity")
    ax.set_title("Light Sensor Readings Over Time")
    ax.legend()
    
    # Update Streamlit chart
    chart_placeholder.pyplot(fig)

    # Close figure to prevent memory leak
    plt.close(fig)

    time.sleep(1)  # 1-second delay for real-time update
