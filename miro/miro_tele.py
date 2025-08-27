#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
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

msg = """
Control Your Miro Robot!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

def checkLinearLimitVelocity(vel):
    return constrain(vel, -MIRO_MAX_LIN_VEL, MIRO_MAX_LIN_VEL)

def checkAngularLimitVelocity(vel):
    return constrain(vel, -MIRO_MAX_ANG_VEL, MIRO_MAX_ANG_VEL)

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('miro_teleop')
    rate = rospy.Rate(50)  # 100 Hz
    pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist_stamped = TwistStamped()
            twist_stamped.header = Header()
            twist_stamped.header.stamp = rospy.Time.now()  # Set the current time
            twist_stamped.twist.linear.x = control_linear_vel
            twist_stamped.twist.angular.z = control_angular_vel

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))

            pub.publish(twist_stamped)
            rate.sleep()

    except Exception as e:
        print("Communications Failed: ", e)

    finally:
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = rospy.Time.now()  # Set the current time
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0
        pub.publish(twist_stamped)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
