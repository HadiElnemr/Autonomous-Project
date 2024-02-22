#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import time
from matplotlib import pyplot as plt

#Initialize ROS Node
rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_18')#, anonymous=True) #Identify ROS Node

V_d = rospy.get_param("/steer_bot/Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_18/V_d")
#W_d= 0 #rospy.get_param("~W_d")
Vactual=0
Wactual=0
linear_v = 0 #Initialize linear velocity 
#angular_v = 0 #Initialize angular velocity
Kp=0.8

# pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
pub = rospy.Publisher('/long_control', Twist, queue_size=10)
vel_msg = Twist()
rate = rospy.Rate(10) # 10 Hz

def callback(data):
  global Vactual
  #global Wactual
  
  Vactual=round(data.twist.twist.linear.x,4)
  #Wactual=round(data.twist.twist.angular.z,4)
  # print('Actual v=',Vactual)

sub = rospy.Subscriber("/steer_bot/ackermann_steering_controller/odom", Odometry, callback)
# sub = rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, callback)
def control():
  global V_d
  #global W_d
  global Vactual
  #global Wactual
  global Vcontrol
 # global Wcontrol
  global Kp
  errorV=V_d-Vactual
  #errorW=W_d-Wactual
  Vcontrol=V_d+Kp*errorV
  #Wcontrol=W_d+Kp*errorW

  


  
time.sleep(5)

### Evaluation KPIs ###
V_actuals = []
times = []

start = rospy.get_rostime()

while 1 and not rospy.is_shutdown():
  control()
  # print(V_d)
  v=round(Vcontrol,2)
 # w=round(Wcontrol,2)
  
  vel_msg.linear.x = v  #Linear Velocity
 # vel_msg.angular.z = w #Angular Velocity
  #ROS Code Publisher
  pub.publish(vel_msg)	#Publish msg
  rate.sleep()		#Sleep with rate

  # V_actuals.append(Vactual)
  # times.append(rospy.get_rostime().secs - start.secs)

  # if len(V_actuals) > 150:
  #   break

plt.plot(times, V_actuals)
plt.axhline(y=V_d, color='r', linestyle='--')

plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')

plt.legend(['Actual Velocity', 'Desired Velocity'])
plt.show()