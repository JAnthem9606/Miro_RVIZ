import rospy
from std_msgs.msg import Float32MultiArray
import streamlit as st

# Initialize ROS Node
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_cosmetic_joints_realtime', anonymous=True, disable_signals=True)

# ROS Publisher
pub = rospy.Publisher('/miro/control/cosmetic_joints', Float32MultiArray, queue_size=10)

# Set Streamlit UI config
st.set_page_config(page_title="Live Cosmetic Joint Controller", layout="centered")
st.title("🎛️ Miro-e Cosmetic Joint Controller")

# Joint configuration
joint_names = ["droop", "wag", "eyel", "eyer", "earl", "earr"]

# Use session state to track previous values
if "joint_values" not in st.session_state:
    st.session_state.joint_values = [0.5] * len(joint_names)

# Function to publish current joint values
def publish_joint_values():
    msg = Float32MultiArray()
    msg.data = st.session_state.joint_values
    pub.publish(msg)

# Create sliders and update ROS topic on change
for i, joint in enumerate(joint_names):
    val = st.slider(
        f"{joint.capitalize()}",
        min_value=0.0,
        max_value=1.0,
        value=st.session_state.joint_values[i],
        step=0.01,
        key=f"{joint}_slider",
        on_change=publish_joint_values
    )

    # Keep session state updated
    st.session_state.joint_values[i] = st.session_state[f"{joint}_slider"]

# Optional: Display current values
st.write("🧾 Current Joint Values:")
st.json({joint: val for joint, val in zip(joint_names, st.session_state.joint_values)})
