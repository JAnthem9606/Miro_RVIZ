import rospy
import streamlit as st
from geometry_msgs.msg import TwistStamped
import time

try:
    rospy.init_node('custom_control_node', anonymous=True, disable_signals=True)
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
    if 'latencies_custom' not in st.session_state:
        st.session_state.latencies_custom = []
    st.session_state.latencies_custom.append(latency)

    avg_latency = sum(st.session_state.latencies_custom) / len(st.session_state.latencies_custom)
    st.write(f"🕒 Last Latency: {latency:.6f} seconds")
    st.write(f"📊 Average Latency: {avg_latency:.6f} seconds")

def control_with_custom_velocity():
    st.subheader("Custom Velocity Control")

    linear_x = st.session_state.get("linear_x", 0.0)
    angular_z = st.session_state.get("angular_z", 0.0)

    lin = st.text_input("Enter Linear Velocity (m/s)", value=str(linear_x))
    ang = st.text_input("Enter Angular Velocity (rad/s)", value=str(angular_z))
    duration = st.text_input("Enter the Duration", value="1", key="custom_duration")

    if st.button("Send"):
        try:
            lin_val = float(lin)
            ang_val = float(ang)
            dur_val = float(duration)

            send_cmd_vel(lin_val, ang_val, dur_val)
            st.success(f"Sent: Linear = {lin_val}, Angular = {ang_val}, Duration = {dur_val} s")
            st.session_state.linear_x = lin_val
            st.session_state.angular_z = ang_val
        except ValueError:
            st.error("Enter valid numeric values.")

if __name__ == "__main__":
    control_with_custom_velocity()
