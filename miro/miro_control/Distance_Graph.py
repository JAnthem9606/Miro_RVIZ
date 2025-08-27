import streamlit as st
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Range

# Initialize ROS Node
rospy.init_node("sonar_streamlit_plot", anonymous=True,disable_signals=True)

# Create a placeholder for live data
st.title("Live Sonar Data Plot")
chart_placeholder = st.empty()

# Store received data
data = pd.DataFrame(columns=["Time", "Distance"])

# Global variable to store sonar data
sonar_range = 0.0

# Callback function to update sonar_range
def sonar_callback(msg):
    global sonar_range
    if msg.range >1:
        sonar_range = 1  # Extract sonar distance
    else:
        sonar_range=msg.range

# Subscribe to ROS topic
rospy.Subscriber("/miro/sensors/sonar", Range, sonar_callback)

# Start time
start_time = time.time()

# Live loop
while not rospy.is_shutdown():
    current_time = time.time() - start_time
    new_row = pd.DataFrame({"Time": [current_time], "Distance": [sonar_range]})
    data = pd.concat([data, new_row], ignore_index=True)

    # Plot the data
    fig, ax = plt.subplots()
    ax.plot(data["Time"], data["Distance"], marker="o", linestyle="-", color="b")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_title("Sonar Distance Over Time")

    # Update Streamlit plot
    chart_placeholder.pyplot(fig)
    plt.close(fig)
    time.sleep(1)  # 1-second delay
    
