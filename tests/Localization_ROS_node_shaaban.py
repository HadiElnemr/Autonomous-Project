#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import time

time.sleep(5)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Localization_Module') #Identify ROS Node

pub1 = rospy.Publisher('/Act_pose', Pose, queue_size=10)
pub2 = rospy.Publisher('/Act_speed', Twist, queue_size=10)
position = Pose()
speed = Twist()
Actual_position = Pose()
Actual_Speed = Twist()
rate = rospy.Rate(10) # 10 Hz
########################################################################

def callback_loc(data):
  global Actual_position,Actual_Speed	#Identify msg variable created as global variable

  Actual_position = data.pose[2]
  Actual_Speed = data.twist[2]

sub = rospy.Subscriber('/steer_bot/gazebo/model_states', ModelStates, callback_loc)
 
while 1 and not rospy.is_shutdown():
  
  position = Actual_position
  speed = Actual_Speed
  #ROS Code Publisher
  pub1.publish(position)	#Publish msg
  pub2.publish(speed)
  rate.sleep()		#Sleep with rate
