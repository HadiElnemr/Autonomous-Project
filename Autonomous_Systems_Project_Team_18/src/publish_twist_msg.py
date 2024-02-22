#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('twist_publisher')
    pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 1.0
        twist_msg.angular.z = -0.01

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
