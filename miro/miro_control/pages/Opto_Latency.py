import rospy
import streamlit as st
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

# ROS Node Initialization
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_light_latency', anonymous=True, disable_signals=True)

# Streamlit Setup
st.set_page_config(page_title="💡 Light Sensor Latency Monitor", layout="centered")
st.title("📊 /sensors/light - Latency Monitor")

# Placeholders
data_placeholder = st.empty()
latency_placeholder = st.empty()

# Variables
latest_light = {"data": None}
latencies = []
frame_count = 0

# Callback function
def light_callback(msg):
    latest_light["data"] = msg.data

# Subscribe only once
if "light_subscribed" not in st.session_state:
    rospy.Subscriber('/miro/sensors/light', Float32MultiArray, light_callback, queue_size=1)
    st.session_state.light_subscribed = True

# Main loop
while not rospy.is_shutdown():
    if latest_light["data"] is not None:
        now = time.time()

        # Display all four light sensor values
        sensor_labels = ["Front Left", "Front Right", "Rear Left", "Rear Right"]
        light_values = np.round(latest_light["data"], 3).tolist()
        display_text = "\n".join([f"{label}: {value}" for label, value in zip(sensor_labels, light_values)])
        data_placeholder.text(f"Latest /sensors/light readings:\n{display_text}")

        end = time.time()
        latency = end - now
        latencies.append(latency)
        frame_count += 1

        if frame_count == 50:
            avg_latency = sum(latencies) / len(latencies)
            latency_placeholder.info(f"🕒 Average Latency over 50 frames: {avg_latency*1000:.3f} ms")
            latencies = []
            frame_count = 0

    time.sleep(0.05)  # small sleep to smooth the updates
