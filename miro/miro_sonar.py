#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import tf
import tf.transformations as tftr

def sonar_callback(data):
    distance = data.range
    print(f'Obstacle Distance : {round(distance,2)} meter')
    
    # Create a quaternion from roll, pitch, yaw (all 0 in this case)
    quaternion = tftr.quaternion_from_euler(0, 0, 0)
    
    broadcaster.sendTransform(
        (0.0, 0.0, 0.0),  # Translation 
        quaternion,        # Rotation as quaternion
        rospy.Time.now(),  # Timestamp
        'nose_link',       # Child frame
        'head_upper'       # Parent frame
    )

if __name__ == '__main__':
    rospy.init_node("Miro_Sonar")
    broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/miro/sensors/sonar", Range, sonar_callback)
    rospy.spin()
