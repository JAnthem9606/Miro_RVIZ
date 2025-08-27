# callbacks.py
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Int16MultiArray

# Global variables
latest_distance = None
latest_light = None
latest_mic = None

def mic_callback(data):
    global latest_mic
    latest_mic = data.data
    latest_mic = max(np.array(latest_mic))

# Callback function for sonar data
def sonar_callback(data):
    global latest_distance
    latest_distance = round(data.range, 2)

def light_sensors_callback(data):
    global latest_light
    lights = list(data.data)
    lights_labels = ['FRONT LEFT : ', 'RIGHT : ', 'REAR LEFT : ', 'RIGHT : ']
    latest_light = (lights_labels[0], round(lights[0], 2),
                    lights_labels[1], round(lights[1], 2),
                    lights_labels[2], round(lights[2], 2),
                    lights_labels[3], round(lights[3], 2))
