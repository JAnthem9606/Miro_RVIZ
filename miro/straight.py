#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header, Float32MultiArray
import sys, select, os

if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios

# Miro Robot Max Velocities
MIRO_MAX_LIN_VEL = 0.4  # max linear velocity
MIRO_MAX_ANG_VEL = 2  # max angular velocity

# Step sizes for control
LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

wheel_effort = [0.0, 0.0]  # Stores last received wheel efforts

def wheel_effort_callback(msg):
    global wheel_effort
    wheel_effort = list(msg.data)  # Store the latest wheel effort values

def adjust_velocity(target_linear_vel):
    global wheel_effort
    if len(wheel_effort) < 2:
        return 0.0  # No correction if data is missing

    left_wheel, right_wheel = wheel_effort
    error = left_wheel - right_wheel
    correction = error * 0.1  # Adjust factor as needed
    return correction

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                return msvcrt.getch().decode() if sys.version_info[0] >= 3 else msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    return max(low, min(input, high))

def checkLinearLimitVelocity(vel):
    return constrain(vel, -MIRO_MAX_LIN_VEL, MIRO_MAX_LIN_VEL)

def checkAngularLimitVelocity(vel):
    return constrain(vel, -MIRO_MAX_ANG_VEL, MIRO_MAX_ANG_VEL)

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('miro_teleop')
    rate = rospy.Rate(50)  # 50 Hz
    pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)
    rospy.Subscriber('/miro/sensors/wheel_effort_pwm', Float32MultiArray, wheel_effort_callback)

    target_linear_vel = 0.0
    target_angular_vel = 0.0

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w':  # Forward
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                target_angular_vel = adjust_velocity(target_linear_vel)
            elif key == 'x':  # Backward
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                target_angular_vel = adjust_velocity(target_linear_vel)
            elif key == 'a':  # Left turn
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
            elif key == 'd':  # Right turn
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
            elif key == ' ' or key == 's':  # Stop
                target_linear_vel = 0.0
                target_angular_vel = 0.0

            twist_stamped = TwistStamped()
            twist_stamped.header = Header()
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.twist.linear.x = target_linear_vel
            twist_stamped.twist.angular.z = target_angular_vel

            pub.publish(twist_stamped)
            rate.sleep()

    except Exception as e:
        print("Error:", e)

    finally:
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0
        pub.publish(twist_stamped)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
