import rospy
from std_msgs.msg import Float32MultiArray
import time
import streamlit as st

# Initialize ROS Node
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_sensor_latency_light', anonymous=True, disable_signals=True)

# Globals
message_times = []

# Callback to record time of message arrival
def callback(data):
    now = time.time()
    message_times.append(now)

# Subscribe once
sub = rospy.Subscriber('/miro/sensors/light', Float32MultiArray, callback)

# Streamlit UI
st.set_page_config(page_title="Sensor Latency", layout="centered")
st.title("💡 Light Sensor Latency Monitor")

if st.button("📡 Start Listening (wait a few messages)"):
    st.info("Listening for sensor messages... Please wait a few seconds.")
    time.sleep(1)  # Collect messages for a short duration

    if len(message_times) >= 2:
        # Compute time differences between consecutive messages
        diffs = [t2 - t1 for t1, t2 in zip(message_times[:-1], message_times[1:])]
        avg_latency = sum(diffs) / len(diffs)
        st.success(f"✅ Estimated Sensor Latency (publish interval): {avg_latency:.6f} sec")
        st.write("📝 Individual Intervals:")
        for i, d in enumerate(diffs, 1):
            st.write(f"Interval {i}: {d:.6f} sec")
    else:
        st.warning("⚠️ Not enough messages received to compute latency.")
