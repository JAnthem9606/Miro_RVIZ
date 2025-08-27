#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import streamlit as st
import time
# Initialize the ROS node
try:
    rospy.init_node('kinematic_joints_publisher', anonymous=True, disable_signals=True)
except rospy.ROSException:
    # Node is already initialized, which is fine
    pass

#latencies = []
st.set_page_config(
    #page_icon=":robot:",
    layout="centered"
)

def main():
    st.title("🤖 Miro-e Kinematics Control")
    st.write("Control the Miro-e robot's joints using the following attributes:")

# Streamlit inputs for joint positions
    tilt = st.slider("Tilt Position", min_value=0.1, max_value=0.9, value=0.1, step=0.1)
    lift = st.slider("Lift Position", min_value=0.1, max_value=0.9, value=0.1, step=0.1)
    yaw = st.slider("Yaw Position", min_value=-1.0, max_value=1.0, value=0.0, step=0.1)
    pitch = st.slider("Pitch Position", min_value=-1.0, max_value=1.0, value=0.0, step=0.1)

# Default values for velocity and effort
    DEFAULT_VELOCITY = 0.1
    DEFAULT_EFFORT = 0.1

# Create a publisher for the /miro/control/kinematic_joints topic
    pub = rospy.Publisher('/miro/control/kinematic_joints', JointState, queue_size=10, latch=True)

# Create and fill the JointState message
    joint_state_msg = JointState()
    joint_state_msg.name = ['tilt', 'lift', 'yaw', 'pitch']
    joint_state_msg.position = [tilt, lift, yaw, pitch]
    joint_state_msg.velocity = 4 * [DEFAULT_VELOCITY]
    joint_state_msg.effort = 4 * [DEFAULT_EFFORT]
    start_time = time.time()
    pub.publish(joint_state_msg)
    
    if 'latencies' not in st.session_state:
        st.session_state.latencies = []

# Publish the joint state

# After publishing
    end_time = time.time()
    latency = end_time - start_time
    st.session_state.latencies.append(latency)

# Calculate average latency
    average_latency = sum(st.session_state.latencies) / len(st.session_state.latencies)

# Display results
    st.write("Latencies length so far: ", len(st.session_state.latencies))
    st.write(f"🕒 Average Latency (Slider to Publish): {average_latency:.6f} seconds")

if __name__ == "__main__":
    main()  
