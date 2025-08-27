import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

# Set Streamlit page layout
st.set_page_config(layout="centered")

st.title("Live Wheel Speed Analysis")
chart_placeholder = st.empty()

# Initialize DataFrame
data = pd.DataFrame(columns=["Time", "Opto Speed Left", "Opto Speed Right", "Back EMF Left", "Back EMF Right"])

# Initialize global variables
wheel_speed_opto = [0, 0]  # [Left, Right]
wheel_speed_back_emf = [0, 0]  # [Left, Right]

# Initialize ROS Node
try:
    rospy.init_node("Wheel_Speed_Live_Plot", disable_signals=True)
except:
    print("No Initialization")

# Callback functions
def wheel_speed_opto_callback(msg):
    global wheel_speed_opto
    wheel_speed_opto = list(msg.data[:2])  # Take only first two values (Left & Right)

def wheel_speed_back_emf_callback(msg):
    global wheel_speed_back_emf
    wheel_speed_back_emf = list(msg.data[:2])  # Take only first two values (Left & Right)

# Subscribe to ROS topics
rospy.Subscriber("/miro/sensors/wheel_speed_opto", Float32MultiArray, wheel_speed_opto_callback)
rospy.Subscriber("/miro/sensors/wheel_speed_back_emf", Float32MultiArray, wheel_speed_back_emf_callback)

# Start time
start_time = time.time()

# Live update loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time

    # Append new data
    new_row = pd.DataFrame({
        "Time": [current_time],
        "Opto Speed Left": [wheel_speed_opto[0]], "Opto Speed Right": [wheel_speed_opto[1]],
        "Back EMF Left": [wheel_speed_back_emf[0]], "Back EMF Right": [wheel_speed_back_emf[1]]
    })
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot data
    fig, ax = plt.subplots()

    # Plot Opto Speeds
    ax.plot(data["Time"], data["Opto Speed Left"], label="Opto Left", marker="o", linestyle="dashed", color="blue",markersize=2.5)
    ax.plot(data["Time"], data["Opto Speed Right"], label="Opto Right", marker="o", linestyle="dashed", color="red",markersize=2.5)

    # Plot Back EMF Speeds
    ax.plot(data["Time"], data["Back EMF Left"], label="Back EMF Left", marker="s", color="blue",markersize=2.5)
    ax.plot(data["Time"], data["Back EMF Right"], label="Back EMF Right", marker="s", color="red",markersize=2.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Encoder Velocity and Back EMF")
    ax.set_title("Wheel Speed Opto vs. Back EMF")
    ax.legend()

    # Update Streamlit chart
    chart_placeholder.pyplot(fig)

    # Close figure to prevent memory leak
    plt.close(fig)

    time.sleep(1)  # 1-second delay for live update
