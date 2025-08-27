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
st.set_page_config(page_title="📷 Camera Feed with Latency & FPS", layout="centered")
st.title("📸 Miro-e Live Camera Feed with Latency and FPS")

# Bridge to convert ROS image to OpenCV
bridge = CvBridge()

# Placeholders for image, latency, and FPS
image_placeholder = st.empty()
latency_placeholder = st.empty()
fps_placeholder = st.empty()

# Shared variable to store latest image
latest_img = {"frame": None}

# Latency and FPS tracking
latencies = []
frame_times = []
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
    rospy.Subscriber('/depth_anything/video/compressed', CompressedImage, image_callback, queue_size=1)
    st.session_state.subscribed = True

# Display live feed and compute latency and FPS every 50 frames
while True:
    if latest_img["frame"] is not None:
        start_time = time.time()

        # Display the image
        image_placeholder.image(latest_img["frame"], channels="RGB", caption="Live Camera Feed")

        end_time = time.time()
        latency = end_time - start_time
        latencies.append(latency)
        frame_times.append(end_time)

        frame_count += 1

        # After every 50 frames
        if frame_count == 50:
            # Latency
            avg_latency = sum(latencies) / len(latencies)
            latency_placeholder.info(f"🕒 Average Latency (last 50 frames): {avg_latency:.6f} seconds")

            # FPS
            elapsed_time = frame_times[-1] - frame_times[0]
            fps = 49 / elapsed_time if elapsed_time > 0 else 0.0
            fps_placeholder.success(f"🎞️ Average FPS (last 50 frames): {fps:.2f}")

            # Reset tracking
            latencies.clear()
            frame_times.clear()
            frame_count = 0

    time.sleep(0.05)  # ~20Hz update rate
