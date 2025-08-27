import rospy
import streamlit as st
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# ROS Node Initialization
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_camera_latency', anonymous=True, disable_signals=True)

# Streamlit App Config
st.set_page_config(page_title="📷 Camera Feed with Latency", layout="centered")
st.title("📸 Miro-e Live Camera Feed with Latency")

# Bridge to convert ROS image to OpenCV
bridge = CvBridge()

# Placeholders for image and latency
image_placeholder = st.empty()
latency_placeholder = st.empty()

# Shared variable to store latest image
latest_img = {"frame": None}

# Latency tracking
latencies = []
frame_count = 0

# Callback function
def image_callback(msg):
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        latest_img["frame"] = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
    except Exception as e:
        st.error(f"Failed to decode image: {e}")

# Subscribe once
if "subscribed" not in st.session_state:
    rospy.Subscriber('/miro/sensors/caml/compressed', CompressedImage, image_callback, queue_size=1)
    st.session_state.subscribed = True

# Display live feed and compute latency every 50 frames
while True:
    if latest_img["frame"] is not None:
        now = time.time()

        # Display the image
        image_placeholder.image(latest_img["frame"], channels="RGB", caption="Live Camera Feed")

        end = time.time()
        latency = end - now
        latencies.append(latency)
        frame_count += 1

        # After every 50 frames
        if frame_count == 50:
            avg_latency = sum(latencies) / len(latencies)
            latency_placeholder.info(f"🕒 Average Latency (last 50 frames): {avg_latency:.6f} seconds")
            latencies = []
            frame_count = 0

    time.sleep(0.05)  # ~20Hz update rate
