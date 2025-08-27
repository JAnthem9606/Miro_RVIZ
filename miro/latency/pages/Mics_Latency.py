import rospy
import streamlit as st
from std_msgs.msg import Int16MultiArray
import numpy as np
import time

# ROS Node Initialization
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_mics_latency', anonymous=True, disable_signals=True)

# Streamlit Setup
st.set_page_config(page_title="🎤 Mics Stream with Latency", layout="centered")
st.title("🎧 Miro-e Microphones - Live Sample and Latency Monitor")

# Streamlit placeholders
mic_placeholder = st.empty()
latency_placeholder = st.empty()

# Variables
latest_mics = {"samples": None}
latencies = []
frame_count = 0

# Callback function for /sensors/mics
def mic_callback(msg):
    data = np.array(msg.data, dtype=np.int16)
    reshaped = data.reshape(-1, 4)  # [LEFT, RIGHT, CENTRE, TAIL]
    latest_mics["samples"] = {
        "LEFT": reshaped[:, 0],
        "RIGHT": reshaped[:, 1],
        "CENTRE": reshaped[:, 2],
        "TAIL": reshaped[:, 3],
    }

# Subscribe once
if "subscribed" not in st.session_state:
    rospy.Subscriber('/miro/sensors/mics', Int16MultiArray, mic_callback, queue_size=1)
    st.session_state.subscribed = True

# Main loop
while True:
    if latest_mics["samples"] is not None:
        print(latest_mics)
        now = time.time()

        # Display first 5 samples of each mic
        output = ""
        for mic_name, samples in latest_mics["samples"].items():
            output += f"🔊 {mic_name}: {samples[:5].tolist()}\n"
        mic_placeholder.text(output)

        end = time.time()
        latency = end - now
        latencies.append(latency)
        frame_count += 1

        if frame_count == 50:
            avg_latency = sum(latencies) / len(latencies)
            latency_placeholder.info(f"🕒 Average Latency (50 frames): {avg_latency:.6f} sec")
            latencies = []
            frame_count = 0

    #time.sleep(0.05)
