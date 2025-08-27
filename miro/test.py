#!/usr/bin/env python
import rospy
import tf2_ros
import tf

def list_frames():
    rospy.init_node('comprehensive_frame_diagnostics')
    
    # TF Listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # TF Broadcaster for additional verification
    broadcaster = tf.TransformBroadcaster()
    
    rospy.sleep(2)  # Give time to collect frames
    
    try:
        # Get all available frames
        rospy.loginfo("Attempting to list all frames:")
        
        # Method 1: tf2_ros buffer
        frames_string = tfBuffer.all_frames_as_string()
        rospy.loginfo("Frames from tf2_ros Buffer:")
        rospy.loginfo(frames_string)
        
        # Method 2: Direct frame listing
        rospy.loginfo("\nAttempting to get frames directly:")
        frame_list = rospy.get_param('/tf_frames', [])
        rospy.loginfo(f"Frames from parameter server: {frame_list}")
        
        # Attempt transform between specific frames
        rospy.loginfo("\nAttempting transform between head_upper and nose_link:")
        try:
            transform = tfBuffer.lookup_transform('head_upper', 'nose_link', rospy.Time())
            rospy.loginfo("Transform found successfully!")
            rospy.loginfo(f"Transform details: {transform}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform lookup failed: {e}")
        
    except Exception as e:
        rospy.logerr(f"Comprehensive diagnostics error: {e}")

if __name__ == '__main__':
    try:
        list_frames()
    except rospy.ROSException as e:
        rospy.logerr(f"ROS Exception: {e}")
    
    rospy.spin()
