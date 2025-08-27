import rospy
from std_msgs.msg import UInt16MultiArray
import time
import streamlit as st

# Initialize ROS Node
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_latency_tone', anonymous=True, disable_signals=True)

# ROS Publisher
pub = rospy.Publisher('/miro/control/tone', UInt16MultiArray, queue_size=10)

# Function to publish tone and measure latency
def measure_latency_tone(freq, volume, duration, trials=5):
    latencies = []

    # Wait for publisher connection
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    for _ in range(trials):
        msg = UInt16MultiArray()
        msg.data = [freq, volume, duration]

        start_time = time.time()
        pub.publish(msg)
        end_time = time.time()

        latencies.append(end_time - start_time)
        time.sleep(0.1)

    return latencies

# Streamlit UI
st.set_page_config(page_title="Tone Latency Monitor", layout="centered")
st.title("🔊 ROS Tone (Speaker) Latency Monitor")

st.subheader("🎼 Tone Parameters")

freq = st.slider("Frequency (Hz)", min_value=50, max_value=2000, value=440, step=10, key="freq")
volume = st.slider("Volume (0-255)", min_value=0, max_value=255, value=100, key="volume")
duration = st.slider("Duration (in 20ms ticks)", min_value=1, max_value=100, value=10, key="duration")

trials = st.number_input("Number of Trials", min_value=1, value=5, key="trials")

if st.button("📤 Send and Measure Latency"):
    latencies = measure_latency_tone(freq, volume, duration, trials)
    avg_latency = sum(latencies) / len(latencies)

    st.success(f"✅ Average Latency over {trials} trials: {avg_latency:.9f} seconds")
    st.write("📝 Individual Latencies:")
    for i, latency in enumerate(latencies, 1):
        st.write(f"Trial {i}: {latency:.9f} sec")
