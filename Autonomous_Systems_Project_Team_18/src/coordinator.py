#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

rospy.init_node('coordinate')#, anonymous=True) #Identify ROS Node

pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
rate = rospy.Rate(10) # 10 Hz
Vactual=0
Wactual=0

def callbackV(data):
  global Vactual
  Vactual=round(data.linear.x,4)
  
 
subV = rospy.Subscriber("/long_control", Twist, callbackV)

def callbackW(data):
  global Wactual
  Wactual=round(data.angular.z,4)
  
subW = rospy.Subscriber("/lat_control", Twist, callbackW)

while 1 and not rospy.is_shutdown():
  vel_msg.linear.x = Vactual
  vel_msg.angular.z = Wactual
  print("Coordinate: Vactual", Vactual, "Wactual", Wactual)
  pub.publish(vel_msg)	#Publish msg
  rate.sleep()		#Sleep with rate
