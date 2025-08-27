import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

# Set up Streamlit layout
st.set_page_config(layout="centered")

st.title("Live Cliff Sensor Data")
chart_placeholder = st.empty()

# Initialize DataFrame
data = pd.DataFrame(columns=["Time", "Cliff Left", "Cliff Right"])

# Global variables for cliff sensor values
cliff_sensors = [0, 0]  # [Left, Right]

# Initialize ROS node
try:
    rospy.init_node("Cliff_Sensors_Live_Plot", disable_signals=True)
except:
    print("No Initialization")

# Callback function
def cliff_sensor_callback(msg):
    global cliff_sensors
    cliff_sensors = list(msg.data[:2])  # Extract first two values (Left & Right)

# Subscribe to the cliff sensor topic
rospy.Subscriber("/miro/sensors/cliff", Float32MultiArray, cliff_sensor_callback)

# Start time
start_time = time.time()

# Live update loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time

    # Append new data
    new_row = pd.DataFrame({
        "Time": [current_time],
        "Cliff Left": [cliff_sensors[0]], "Cliff Right": [cliff_sensors[1]]
    })
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot data
    fig, ax = plt.subplots()

    ax.plot(data["Time"], data["Cliff Left"], label="Cliff Left", marker="o",  markersize=2.5)
    ax.plot(data["Time"], data["Cliff Right"], label="Cliff Right", marker="o",markersize=2.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cliff Sensor Value")
    ax.set_title("Cliff Sensor Data (Left & Right)")
    ax.legend()

    # Update Streamlit chart
    chart_placeholder.pyplot(fig)

    # Close figure to prevent memory leak
    plt.close(fig)

    time.sleep(1)  # 1-second delay for live update
