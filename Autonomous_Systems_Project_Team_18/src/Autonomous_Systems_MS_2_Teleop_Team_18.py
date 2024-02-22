#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import sys, select, termios, tty, math

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : stop
CTRL-C to quit
"""

def QtoE(x,y,z,w):
    t0=+2.0*(w*z+x*y)
    t1=+1.0-2.0*(y*y+z*z)
    yaw=math.atan2(t0,t1)
    return yaw 

def callback(data):
    #rospy.loginfo("\r (Teleop node) Rear Wheel Joint Position: " + str(round(data.position[-1], 6)) + ' \r ')
    rospy.loginfo("\r (Teleop node): x: " + str(round(data.pose.pose.position.x, 6)) + "    y: " + 
            str(round(data.pose.pose.position.y, 6))  + '\n\r\n')
   
    orientation_q = data.pose.pose.orientation
    orientation_e = QtoE(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    rospy.loginfo("\r (Teleop node): Orientation Angle: " + str(orientation_e) + '\n\r\n')

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(linear_vel, angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel, angular_vel)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber("/steer_bot/ackermann_steering_controller/odom", Odometry, callback)

    status = 0
    linear_vel = 0.0
    angular_vel = 0.0
    try:

        while(1):
            key = getKey()
            if key == 'w':
                linear_vel += 0.1
                status = status + 1
            elif key == 'x':
                linear_vel -= 0.1
                status = status + 1
            elif key == 'a':
                angular_vel += 0.1
                status = status + 1
            elif key == 'd':
                angular_vel -= 0.1
                status = status + 1
            elif key == ' ' or key == 's':
                linear_vel = 0.0
                angular_vel = 0.0 
            else:
                if (key == '\x03'):
                    break
            linear_vel = math.copysign(min(abs(linear_vel), 8), linear_vel)
            angular_vel = math.copysign(min(abs(angular_vel), 45/180*math.pi), angular_vel)
            #print(30/180*math.pi)
            #print(vels(linear_vel,angular_vel))
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel 
            twist_msg.angular.z = angular_vel 
            pub.publish(twist_msg)
            rospy.loginfo("\r (Teleop node): Steering Angle: " + str(angular_vel) + ' \n\r\n')

    except Exception as e:
        print(e)

    finally:
        twist_msg.linear.x=0.
        twist_msg.angular.z=0.
        pub.publish(twist_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
