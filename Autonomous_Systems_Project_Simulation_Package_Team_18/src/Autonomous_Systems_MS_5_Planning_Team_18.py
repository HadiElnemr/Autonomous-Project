#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Twist
import numpy as np
import math
import time

time.sleep(5)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Planning_Module') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Longitudinal_Driving_Velocity', Float64, queue_size=10)
pub2 = rospy.Publisher('/Lateral_Distance', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
Actual_position = Pose()
Actual_Speed = Twist()
flag_Stp = 0
flag_Stv = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Act_Stp(data):
 global Actual_position	#Identify msg variable created as global variable
 global sub1,flag_Stp		#Identify a subscriber as global variable
 
 if flag_Stp == 0:
  Actual_position = data
  flag_Stp =1
  return
 

sub1 = rospy.Subscriber('/Act_pose', Pose, callback_Act_Stp)

def callback_Act_Sts(data):
 global Actual_Speed	#Identify msg variable created as global variable
 global sub1,flag_Stv		#Identify a subscriber as global variable
 
 if flag_Stv == 0:
  Actual_Speed = data
  flag_Stv = 1
  return
 

sub2 = rospy.Subscriber('/Act_speed', Twist, callback_Act_Sts)
#######################################################################
#########################################################################################################

#########################################################################################################
def quaternion_to_euler(x, y, z, w):
 t0 = +2.0 * (w * x + y * z)
 t1 = +1.0 - 2.0 * (x * x + y * y)
 roll = math.atan2(t0, t1)
 t2 = +2.0 * (w * y - z * x)
 t2 = +1.0 if t2 > +1.0 else t2
 t2 = -1.0 if t2 < -1.0 else t2
 pitch = math.asin(t2)
 t3 = +2.0 * (w * z + x * y)
 t4 = +1.0 - 2.0 * (y * y + z * z)
 yaw = math.atan2(t3, t4)
 return [yaw, pitch, roll]


#########################################################################################################
ROAD_WIDTH = 2.4
LANE_WIDTH = ROAD_WIDTH / 2
LEFT_LANE_CENTRE = LANE_WIDTH / 2
RIGHT_LANE_CENTRE = -LANE_WIDTH / 2
OBSTACLE_1_POS = 8
OBSTACLE_2_POS = 16
MANEUVERING_DISTANCE_THRESHOLD = 4
V_MAX = 2 
ROAD_LENGTH = 20

#########################################################################################################

def get_desired_velocity_and_lane(x):
  if OBSTACLE_1_POS - x > MANEUVERING_DISTANCE_THRESHOLD: # car before obs 1
    V_Des = V_MAX
    Lane_Des = LEFT_LANE_CENTRE
  elif abs(OBSTACLE_1_POS - x) < MANEUVERING_DISTANCE_THRESHOLD: # car before or after obs 1 and turning
    V_Des = V_MAX/2 if OBSTACLE_1_POS - x > 0 else V_MAX
    Lane_Des = RIGHT_LANE_CENTRE
  elif OBSTACLE_2_POS - x > MANEUVERING_DISTANCE_THRESHOLD:
    V_Des = V_MAX
    Lane_Des = RIGHT_LANE_CENTRE
  elif abs(OBSTACLE_2_POS - x) < MANEUVERING_DISTANCE_THRESHOLD:
    V_Des = V_MAX/2
    Lane_Des = LEFT_LANE_CENTRE
  elif ROAD_LENGTH - x < 6:
    V_Des = 0
    Lane_Des = LEFT_LANE_CENTRE
  else:
    V_Des = 0
    Lane_Des = LEFT_LANE_CENTRE
  return V_Des, Lane_Des

#######################################################################
V_Des = 2
Lane_Des = LEFT_LANE_CENTRE
#######################################################################
#Simulation While Loop
st_time = time.time()
#######################################################################
while 1 and not rospy.is_shutdown():
 st_time = time.time()
 
 if flag_Stp == 1 and flag_Stv == 1:
  V_Act = Actual_Speed.linear.x
 
  Angles_Act = quaternion_to_euler(Actual_position.orientation.x, Actual_position.orientation.y, Actual_position.orientation.z, Actual_position.orientation.w)
  Yaw_act = Angles_Act[0] 
  Pos_Act = [Actual_position.position.x, Actual_position.position.y,Yaw_act]
 
  flag_Stp = 0  
  flag_Stv = 0 
 V_Des, Lane_Des = get_desired_velocity_and_lane(Actual_position.position.x)

 pub1.publish(V_Des)	#Publish msg
 pub2.publish(Lane_Des)	#Publish msg
 rate.sleep()		#Sleep with rate
 end_time = time.time()
 tau  = end_time-st_time
#######################################################################
#########################################################################################################




