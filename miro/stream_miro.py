#!/usr/bin/env python
import rospy
import streamlit as st
from sensor_msgs.msg import JointState
import signal
import sys

# Global publisher
pub = None

# Initialize ROS node before Streamlit
rospy.init_node('kinematic_joints_publisher', anonymous=True, disable_signals=True)


def main():
    global pub
    
    st.title("Miro-e Kinematic Joint Control")
    st.write("Control the kinematic joints of Miro-e robot using the sliders below")
    
    pub = rospy.Publisher('/miro/control/kinematic_joints', JointState, queue_size=10, latch=True)
    vel,eff = 0.1,0.1
    # Create sliders for joint positions
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("Head Control")
        tilt = st.slider("Tilt", -1.0, 1.0, 0.0, 0.01)
        lift = st.slider("Lift", -1.0, 1.0, 0.0, 0.01)
    
    with col2:
        st.subheader("Neck Control")
        yaw = st.slider("Yaw", -1.0, 1.0, 0.0, 0.01)
        pitch = st.slider("Pitch", -1.0, 1.0, 0.0, 0.01)
    
    # Display current joint positions
    st.subheader("Current Joint Positions")
    positions_df = {
        'Joint': ['Tilt', 'Lift', 'Yaw', 'Pitch'],
        'Position': [tilt, lift, yaw, pitch]
    }
    st.table(positions_df)
    
    # Create and fill the JointState message
    joint_state_msg = JointState()
    joint_state_msg.name = ['tilt', 'lift', 'yaw', 'pitch']
    joint_state_msg.position = [tilt, lift, yaw, pitch]
    joint_state_msg.velocity = 4 * [vel]
    joint_state_msg.effort = 4 * [eff]
    
    # Auto-publish the joint states whenever values change

    pub.publish(joint_state_msg)
    
    # Add safety stop button
    if st.button("Emergency Stop", type="primary"):
        try:
            # Send zero position commands
            joint_state_msg.position = 4 * [0.0]
            joint_state_msg.velocity = 4 * [0.0]
            joint_state_msg.effort = 4 * [0.0]
            pub.publish(joint_state_msg)
            st.warning("Emergency stop activated - All joints set to zero position")
        except Exception as e:
            st.error(f"Error during emergency stop: {str(e)}")

    # Add status indicator
    if rospy.is_shutdown():
        st.error("ROS connection lost!")
    else:
        st.success("ROS connection active")

if __name__ == "__main__":
    main()
