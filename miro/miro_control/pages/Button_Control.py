import rospy
import streamlit as st
from geometry_msgs.msg import TwistStamped
import time

try:
    rospy.init_node('button_control_node', anonymous=True, disable_signals=True)
except rospy.ROSException:
    pass

pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)

def send_cmd_vel(linear_x, angular_z, duration):
    twist_stamped = TwistStamped()
    twist_stamped.header.stamp = rospy.Time.now()
    twist_stamped.header.frame_id = "base_link"
    twist_stamped.twist.linear.x = linear_x
    twist_stamped.twist.angular.z = angular_z

    start_time = time.time()
    for _ in range(int(duration * 10)):
        pub.publish(twist_stamped)
        time.sleep(0.1)
    end_time = time.time()

    latency = end_time - start_time
    if 'latencies_btn' not in st.session_state:
        st.session_state.latencies_btn = []
    st.session_state.latencies_btn.append(latency)

    avg_latency = sum(st.session_state.latencies_btn) / len(st.session_state.latencies_btn)
    st.write(f"🕒 Last Latency: {latency:.6f} seconds")
    st.write(f"📊 Average Latency: {avg_latency:.6f} seconds")

def control_with_buttons():
    st.write("# Miro-e Dashboard for Controlling Velocities 🏎️💨")
    col1, col2, col3 = st.columns(3)
    st.subheader("Control Using Buttons")

    with col1:
        forward = st.button("↑ Forward")
        backward = st.button("↓ Backward")
    with col2:
        left = st.button("← Left")
        right = st.button("→ Right")

    duration = st.text_input("Enter the Duration", value="1", key="btn_duration")

    linear_x = 0.0
    angular_z = 0.0

    if forward:
        linear_x = 0.5
        st.success(f"Moving forward for {duration} seconds")
    elif backward:
        linear_x = -0.5
        st.success(f"Moving backward for {duration} seconds")
    elif left:
        angular_z = 0.5
        st.success(f"Turning left for {duration} seconds")
    elif right:
        angular_z = -0.5
        st.success(f"Turning right for {duration} seconds")

    if forward or backward or left or right:
        send_cmd_vel(linear_x, angular_z, float(duration))

    with col3:
        st.write("Linear Velocity (m/s):", linear_x)
        st.write("Angular Velocity (rad/s):", angular_z)

if __name__ == "__main__":
    control_with_buttons()
