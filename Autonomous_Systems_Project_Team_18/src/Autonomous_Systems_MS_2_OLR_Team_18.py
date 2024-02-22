#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
import math


def QtoE(x,y,z,w):
    t0=+2.0*(w*z+x*y)
    t1=+1.0-2.0*(y*y+z*z)
    yaw=math.atan2(t0,t1)
    return yaw

def callback(data):

    #rospy.loginfo("(OLR node) Wheel Joint Position: " + str(data.position[-1]))
    rospy.loginfo(" (OLR node): x: " + str(round(data.pose.pose.position.x, 6)) + "    y: " + 
            str(round(data.pose.pose.position.y, 6)) + '\n\n')
   
    orientation_q = data.pose.pose.orientation
    orientation_e = QtoE(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    rospy.loginfo("(OLR node): Orientation Angle: " + str(orientation_e) + '\n\n')
    
    
def main():
    rospy.init_node('Autonomous_Systems_MS_2_OLR_Team_18')
    
    pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber("/steer_bot/ackermann_steering_controller/odom", Odometry, callback)


    rate = rospy.Rate(10) # 10 Hz
    x = 1.0
    omega = -1.0
   
    if rospy.has_param('/linear_x') and rospy.has_param('/omega'):
        x = rospy.get_param('/linear_x')
        omega = rospy.get_param('/omega')
    

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = 0.0 
        twist_msg.linear.z = 0.0 
        twist_msg.angular.x = 0.0 
        twist_msg.angular.y = 0.0 
        twist_msg.angular.z = omega
        rospy.loginfo("(OLR node): Steering Angle: " + str(omega) + ' \n\n')

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
