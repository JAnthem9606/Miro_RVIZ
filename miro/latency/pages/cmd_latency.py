import rospy
from geometry_msgs.msg import TwistStamped
import time
import streamlit as st

# Initialize ROS Node
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_latency_twiststamped', anonymous=True, disable_signals=True)

# ROS Publisher
pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)

# Function to publish TwistStamped and measure latency
def measure_latency_twiststamped(linear_x, angular_z, trials=5):
    latencies = []

    # Ensure publisher is ready
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    for i in range(trials):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = linear_x
        twist_stamped.twist.angular.z = angular_z

        start_time = time.time()
        pub.publish(twist_stamped)
        end_time = time.time()
        
        latencies.append(end_time - start_time)
        time.sleep(0.1)

    return latencies

# Streamlit UI
st.set_page_config(page_title="TwistStamped Latency Monitor", layout="centered")
st.title("📡 ROS cmd_vel (TwistStamped) Latency Monitor")

# Inputs
linear_x = st.number_input("Linear Velocity (m/s)", value=0.5)
angular_z = st.number_input("Angular Velocity (rad/s)", value=0.0)
trials = st.number_input("Number of Trials", min_value=1, value=5)

# Trigger
if st.button("📤 Send and Measure Latency"):
    latencies = measure_latency_twiststamped(linear_x, angular_z, trials)
    avg_latency = sum(latencies) / len(latencies)

    st.success(f"✅ Average Latency over {trials} trials: {avg_latency:.9f} seconds")
    st.write("📝 Individual Latencies:")
    for i, latency in enumerate(latencies, 1):
        st.write(f"Trial {i}: {latency:.9f} sec")

# ROS Publisher


