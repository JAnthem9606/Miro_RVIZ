# main.py

import streamlit as st

# Titles of the two parts of the app
st.title("Miro Robotics Control and Sensors")

# Create a sidebar with navigation options
option = st.sidebar.selectbox(
    "Select a Section",
    ("Home", "Actuator", "Sensors")
)

# Conditional rendering based on the selected option
if option == "Home":
    st.write("Welcome to the Miro Robotics Control and Sensors Interface!")
    st.write("Choose an option from the sidebar to explore Actuator or Sensors data.")
elif option == "Actuator":
    st.write("You are now in the Actuator section.")
    # Import and run the actuator script
    import miro_cmd_vel  # This will run the miro_cmd_vel.py file
elif option == "Sensors":
    st.write("You are now in the Sensors section.")
    # Import and run the sensors script
    import data  # This will run the data.py file
