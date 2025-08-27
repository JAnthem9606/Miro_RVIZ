import rospy
import streamlit as st
import time
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import UInt16
import rospy

st.set_page_config(layout="centered")

st.title("Live Touch Sensors Data_Plot")
chart_placeholder = st.empty()

data = pd.DataFrame(columns=["Time","Head_Touch","Body_Touch"])
head_touch,body_touch = [0,0]

start_time = time.time()

try:
	rospy.init_node("Touch_Sensors_Live_Data",disable_signals=True)
except:
	print("No Initilization")
def head_callback(data):
	global head_touch
	head_touch = data.data/1000

def body_callback(data):
	global body_touch
	body_touch = data.data/1000

rospy.Subscriber("/miro/sensors/touch_body", UInt16, body_callback)
rospy.Subscriber("/miro/sensors/touch_head", UInt16, head_callback)

while not rospy.is_shutdown():
	current_time = time.time() - start_time
	new_row = pd.DataFrame({"Time":[current_time],"Head_Touch":[head_touch],"Body_Touch":[body_touch]})
	data = pd.concat([data, new_row], ignore_index=True)
	
	fig, ax = plt.subplots()
	
	ax.plot(data["Time"],data["Head_Touch"],label="Head Touch", marker="o",markersize=2.5)
	ax.plot(data["Time"],data["Body_Touch"],label="Body Touch", marker="o",markersize=2.5)
	
	ax.set_xlabel("Time (S)")
	ax.set_ylabel("Touch Values")
	ax.set_title("Head and Body Touch Data Plot")
	ax.legend()
	
	chart_placeholder.pyplot(fig)
	plt.close(fig)
	time.sleep(1)  # 1-second delay for real-time update
	
