import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu

# Set Streamlit page layout
st.set_page_config(layout="centered")

st.title("Live IMU Data Plot (Body & Head)")
chart_placeholder = st.empty()

# Initialize DataFrame
data = pd.DataFrame(columns=["Time", "Body Angular X", "Body Angular Y", "Body Angular Z",
                             "Head Angular X", "Head Angular Y", "Head Angular Z"])

# Initialize global IMU values
body_imu = [0.0, 0.0, 0.0]
head_imu = [0.0, 0.0, 0.0]

# Initialize ROS Node
try:
    rospy.init_node("IMU_Live_Data", disable_signals=True)
except:
    print("No Initialization")

# Callback functions
def imu_body_callback(data):
	global body_imu
	body_imu = round(data.linear_acceleration.x, 2),round(data.linear_acceleration.y, 2),round(data.linear_acceleration.z, 2)
	

    # IMU Head Callback
def imu_head_callback(data):
	global head_imu
	head_imu = round(data.linear_acceleration.x, 2),round(data.linear_acceleration.y, 2),round(data.linear_acceleration.z, 2)
	

# Subscribe to ROS topics
rospy.Subscriber("/miro/sensors/imu_body", Imu, imu_body_callback)
rospy.Subscriber("/miro/sensors/imu_head", Imu, imu_head_callback)

# Start time
start_time = time.time()

# Live update loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time

    # Append new data
    new_row = pd.DataFrame({
        "Time": [current_time],
        "Body Angular X": [body_imu[0]],
        "Body Angular Y": [body_imu[1]],
        "Body Angular Z": [body_imu[2]],
        "Head Angular X": [head_imu[0]],
        "Head Angular Y": [head_imu[1]],
        "Head Angular Z": [head_imu[2]]
    })
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot data
    fig, ax = plt.subplots()
    ax.plot(data["Time"], data["Body Angular X"], label="Body Angular X", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Body Angular Y"], label="Body Angular Y", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Body Angular Z"], label="Body Angular Z", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Head Angular X"], label="Head Angular X", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Head Angular Y"], label="Head Angular Y", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Head Angular Z"], label="Head Angular Z", marker="o",markersize=2.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Velocity (rad/s)")
    ax.set_title("IMU Angular Velocity Over Time")
    ax.legend()

    # Update Streamlit chart
    chart_placeholder.pyplot(fig)

    # Close figure to prevent memory leak
    plt.close(fig)

    time.sleep(1)  # 1-second delay for live update
