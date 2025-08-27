import rospy
from std_msgs.msg import UInt32MultiArray
import time
import streamlit as st

# Initialize ROS Node
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_latency_illum', anonymous=True, disable_signals=True)

# ROS Publisher
pub = rospy.Publisher('/miro/control/illum', UInt32MultiArray, queue_size=10)

# Function to publish UInt32MultiArray and measure latency
def measure_latency_illum(values, trials=5):
    latencies = []

    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    for _ in range(trials):
        msg = UInt32MultiArray()
        msg.data = values

        start_time = time.time()
        pub.publish(msg)
        end_time = time.time()

        latencies.append(end_time - start_time)
        time.sleep(0.1)

    return latencies

# Streamlit UI
st.set_page_config(page_title="Illumination Latency", layout="centered")
st.title("💡 ROS Illumination LED Latency Monitor")

st.subheader("🎨 LED bRGB Values (as UInt32)")

default_vals = [0xFFFFFFFF] * 6
illum_values = []

labels = ["Left Front", "Middle 1", "Rear 1", "Right Front", "Middle 2", "Rear 2"]

for i, label in enumerate(labels):
    value = st.text_input(f"{label} bRGB (hex, e.g. FFFFFFFF)", value=f"{default_vals[i]:08X}", key=f"led_input_{i}")
    try:
        illum_values.append(int(value, 16))
    except ValueError:
        illum_values.append(0xFFFFFFFF)

trials = st.number_input("Number of Trials", min_value=1, value=5)

if st.button("📤 Send and Measure Latency"):
    latencies = measure_latency_illum(illum_values, trials)
    avg_latency = sum(latencies) / len(latencies)

    st.success(f"✅ Average Latency over {trials} trials: {avg_latency:.9f} seconds")
    st.write("📝 Individual Latencies:")
    for i, latency in enumerate(latencies, 1):
        st.write(f"Trial {i}: {latency:.9f} sec")
