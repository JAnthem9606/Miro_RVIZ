import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

# Set Streamlit page layout
st.set_page_config(layout="centered")

st.title("Live Wheel Speed Data Plot")
chart_placeholder = st.empty()

# Initialize DataFrame
data = pd.DataFrame(columns=["Time", "Left Wheel", "Right Wheel"])
left_wheel, right_wheel = 0.0, 0.0

# Initialize ROS Node
try:
    rospy.init_node("Wheel_Speed_Live_Data", disable_signals=True)
except:
    print("No Initialization")

# Callback function
def wheel_speed_opto_callback(msg):
    global left_wheel, right_wheel
    wheel_speeds = list(msg.data)  # Extract wheel speeds
    if len(wheel_speeds) >= 2:
        left_wheel, right_wheel = wheel_speeds[:2]  # Assign first two values

# Subscribe to ROS topic
rospy.Subscriber("/miro/sensors/wheel_speed_opto", Float32MultiArray, wheel_speed_opto_callback)

# Start time
start_time = time.time()

# Live loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time

    # Append new data
    new_row = pd.DataFrame({
        "Time": [current_time],
        "Left Wheel": [left_wheel],
        "Right Wheel": [right_wheel]
    })
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot data
    fig, ax = plt.subplots()
    ax.plot(data["Time"], data["Left Wheel"], label="Left Wheel Speed", marker="o")
    ax.plot(data["Time"], data["Right Wheel"], label="Right Wheel Speed", marker="o")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Wheel Speed")
    ax.set_title("Wheel Speed Data Over Time")
    ax.legend()

    # Update Streamlit plot
    chart_placeholder.pyplot(fig)

    # Close figure to prevent memory leak
    plt.close(fig)

    time.sleep(1)  # 1-second delay for live update
