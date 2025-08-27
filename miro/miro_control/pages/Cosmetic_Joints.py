import rospy
from std_msgs.msg import Float32MultiArray
import time
import streamlit as st

# Initialize ROS Node
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_latency_cosmetic', anonymous=True, disable_signals=True)

# ROS Publisher
pub = rospy.Publisher('/miro/control/cosmetic_joints', Float32MultiArray, queue_size=10)

# Function to publish FloatMultiArray and measure latency
def measure_latency_cosmetic_joints(values, trials=5):
    latencies = []

    # Ensure publisher is ready
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    for i in range(trials):
        msg = Float32MultiArray()
        msg.data = values

        start_time = time.time()
        pub.publish(msg)
        end_time = time.time()

        latencies.append(end_time - start_time)
        time.sleep(0.1)

    return latencies

# Streamlit UI
st.set_page_config(page_title="Cosmetic Joints Latency", layout="centered")
st.title("🎭 ROS Cosmetic Joints Latency Monitor")

# Inputs for 6 cosmetic joints
joint_names = ["droop", "wag", "eyel", "eyer", "earl", "earr"]
default_values = [0.5] * 6
user_values = []

st.subheader("🔧 Set Joint Positions (normalized [0.0 – 1.0])")
for i, joint in enumerate(joint_names):
    value = st.slider(f"{joint.capitalize()}", 0.0, 1.0, default_values[i], step=0.01)
    user_values.append(value)

# Trials input
trials = st.number_input("Number of Trials", min_value=1, value=5)

# Trigger
if st.button("📤 Send and Measure Latency"):
    latencies = measure_latency_cosmetic_joints(user_values, trials)
    avg_latency = sum(latencies) / len(latencies)

    st.success(f"✅ Average Latency over {trials} trials: {avg_latency:.9f} seconds")
    st.write("📝 Individual Latencies:")
    for i, latency in enumerate(latencies, 1):
        st.write(f"Trial {i}: {latency:.9f} sec")
