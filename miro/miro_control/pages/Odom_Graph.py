import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

# Set Streamlit page layout
st.set_page_config(layout="centered")

st.title("Live Odometry Data Plot")
chart_placeholder = st.empty()

# Initialize DataFrame
data = pd.DataFrame(columns=["Time", "X Position", "Y Position", "Linear Velocity", "Angular Velocity"])

# Initialize global odometry values
x_pos, y_pos = 0.0, 0.0
linear_velocity, angular_velocity = 0.0, 0.0

# Initialize ROS Node
try:
    rospy.init_node("Odometry_Live_Data", disable_signals=True)
except:
    print("No Initialization")

# Callback function
def odom_callback(msg):
    global x_pos, y_pos, linear_velocity, angular_velocity
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    linear_velocity = msg.twist.twist.linear.x
    angular_velocity = msg.twist.twist.angular.z

# Subscribe to ROS topic
rospy.Subscriber("/miro/sensors/odom", Odometry, odom_callback)

# Start time
start_time = time.time()

# Live update loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time

    # Append new data
    new_row = pd.DataFrame({
        "Time": [current_time],
        "X Position": [x_pos],
        "Y Position": [y_pos],
        "Linear Velocity": [linear_velocity],
        "Angular Velocity": [angular_velocity]
    })
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot data
    fig, ax = plt.subplots()
    ax.plot(data["Time"], data["X Position"], label="X Position", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Y Position"], label="Y Position", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Linear Velocity"], label="Linear Velocity", marker="o",markersize=2.5)
    ax.plot(data["Time"], data["Angular Velocity"], label="Angular Velocity", marker="o",markersize=2.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Odometry Values")
    ax.set_title("Odometry Data Over Time")
    ax.legend()

    # Update Streamlit chart
    chart_placeholder.pyplot(fig)

    # Close figure to prevent memory leak
    plt.close(fig)

    time.sleep(1)  # 1-second delay for live update
