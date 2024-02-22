#! /usr/bin/env python 
# A basic python code to subscribe from the turtlebot its position and orientation through odometery sensor

### Libraries to be imported
import rospy #library to connect ROS with python
import matplotlib.pyplot as plt #library used for plotting 
import numpy as np #library used for mathematical operations
import math
import random
import tf #library used for states transformation from Quaternian to Euler and vice versa

from nav_msgs.msg import Odometry #import msg data type "Odometry" from nav_msgs dependency to be subscribed 
from geometry_msgs.msg import Twist, Pose #import msg data type "Twist" and "Pose" from geometry_msgs dependency to be published and subscribed
####################################################################################################################################################################
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
### Initialize ROS Node 
rospy.init_node('Odometry_Check', anonymous = True) #Identify Ros Node
####################################################################################################################################################################
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
## ROS Publisher Code
pub1 = rospy.Publisher('/Odom_Values', Odometry, queue_size=10) #Identify the publisher "pub1" to publish on topic "/Odom_Values" to send message of type "Odometry"
Odom_Noise = Odometry() #Identify msg variable of data type Odometry
rate = rospy.Rate(10) # rate of publishing msg 10hz
####################################################################################################################################################################
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]
####################################################################################################################################################################
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
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
####################################################################################################################################################################
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
#ROS Subscriber Code for Position
flag_cont = 0			#Initialize flag by zero
pos_msg = Pose()		#Identify msg variable of data type Pose
position = np.zeros((1,6))	#Identify array of six elements all initialized by zero
Velocity_msg = Twist()		#Identify msg variable of data type Twist
velocity = np.zeros((1,6))	#Identify array of six elements all initialized by zero
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
	  global pos_msg	#Identify msg variable created as global variable
	  global sub2		#Identify a subscriber as global variable
	  
	  #Identify all variables as globals 
	  global flag_cont
	  global position 
	  global Velocity_msg
	  global velocity
	  global subcount


	  msg = data ##Extract the data sent as message

	  pos_msg.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
	  pos_msg.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
	  pos_msg.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
	  pos_msg.orientation.x = round(msg.pose.pose.orientation.x, 4)		#Round the value of theta to 4 decimal places
	  pos_msg.orientation.y = round(msg.pose.pose.orientation.y, 4)		#Round the value of theta to 4 decimal places
	  pos_msg.orientation.z = round(msg.pose.pose.orientation.z, 4)		#Round the value of theta to 4 decimal places
	  pos_msg.orientation.w = round(msg.pose.pose.orientation.w, 4)		#Round the value of theta to 4 decimal places
	  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w) #Transform from Quaternion to Euler coordinates
	  ## Another way to transform from Quaternion to Euler
	  ## (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w) #Transform Quaternian to Euler angles
	  position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll] ##Store the position in array

	  Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)		#Round the value of x to 4 decimal places
	  Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)		#Round the value of y to 4 decimal places
	  Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)		#Round the value of z to 4 decimal places
	  Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)		#Round the value of x to 4 decimal places
	  Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)		#Round the value of y to 4 decimal places
	  Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)		#Round the value of z to 4 decimal places
	  velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]##Store the velocity in array
	  
	  Time = msg.header.stamp.to_sec() 	#Extract the time of the simulation
	  
	  flag_cont = 1				#Set flag to one
	  #print("sub" + str(position[0]))

sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	      #Identify msg variable of data type Pose
position_0 = np.zeros((1,6))  #Identify array of six elements all initialized by zero
flag_initial = 0	      #Initialize flag by zero
Velocity_msg_0 = Twist()      #Identify msg variable of data type Pose
velocity_0 = np.zeros((1,6))  #Identify array of six elements all initialized by zero
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Init(data):
	  global pos_msg_0		#Identify msg variable created as global variable
	  global sub1			#Identify a subscriber as global variable

	  #Identify all variables as globals 
	  global flag_initial 	
	  global position_0 
	  global Velocity_msg_0
	  global velocity_0

	  msg = data ##Extract the data sent as message

	  pos_msg_0.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
	  pos_msg_0.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
	  pos_msg_0.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
	  pos_msg_0.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
	  pos_msg_0.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
	  pos_msg_0.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
	  pos_msg_0.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
	  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)#Transform from Quaternion to Euler coordinates
	  ## Another way to transform from Quaternion to Euler
	  ## (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w) #Transform Quaternian to Euler angles
	  position_0 = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]##Store the position in array

	  Velocity_msg_0.linear.x = round(msg.twist.twist.linear.x, 4)		#Round the value of x to 4 decimal places
	  Velocity_msg_0.linear.y = round(msg.twist.twist.linear.y, 4)		#Round the value of y to 4 decimal places
	  Velocity_msg_0.linear.z = round(msg.twist.twist.linear.z, 4)		#Round the value of z to 4 decimal places
	  Velocity_msg_0.angular.x = round(msg.twist.twist.angular.x, 4)	#Round the value of x to 4 decimal places
	  Velocity_msg_0.angular.y = round(msg.twist.twist.angular.y, 4)	#Round the value of y to 4 decimal places
	  Velocity_msg_0.angular.z = round(msg.twist.twist.angular.z, 4)	#Round the value of z to 4 decimal places
	  velocity_0 = [Velocity_msg_0.linear.x,Velocity_msg_0.linear.y,Velocity_msg_0.linear.z,Velocity_msg_0.angular.x,Velocity_msg_0.angular.y,Velocity_msg_0.angular.z]##Store the velocity in array
	  
	  Time = msg.header.stamp.to_sec()

	  flag_initial = 1			#Set the flag to one
	  sub1.unregister()			#Unsubsribe from this topic

sub1 = rospy.Subscriber('/odom', Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
####################################################################################################################################################################
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
  pass
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
##Create an array to have the noisy measurments
position_noisy = np.zeros((1,6)) #Create empty array
## Standard Deviation Inputs
#STD = [rospy.get_param("~xstd"),rospy.get_param("~ystd"),rospy.get_param("~thetastd")]
####################################################################################################################################################################
####################################################################################################################################################################
##Create method to add noise to the sensor
def noisy_add(position, velocity):
	#Define the global variables represented as the standard deviation of each state
	global position_noisy
	global XNoisy
	global YNoisy
	global ZNoisy
	global ThetaNoisy

	xnoise = position[0] + random.uniform(-0.1,0.1)		##Noisy data calculated at x
	ynoise = position[1] + random.uniform(-0.1,0.1)		##Noisy data calculated at y
	thetanoise = position[3] + random.uniform(-0.1,0.1)	##Noisy data calculated at z

	position_noisy = [xnoise, ynoise, position[2], thetanoise, position[4], position[5]]##Store the noisy position in array))
	#print("noisysub" + str(position[0]))
	#print("noisypub" + str(position_noisy[0]))
	return(position_noisy, velocity)			#Return the noisy position and the velocity
####################################################################################################################################################################
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
while 1 and not rospy.is_shutdown():
## If the subscriber subscribes a signal the method "noisy_add" should be called to add noise to the signal subscribed
	if flag_cont == 1:
		[position_noisy, velocity_noisy] = noisy_add(position, velocity)
		flag_cont = 0

	try:
		Odom_Noise.pose.pose.position.x = round(position_noisy[0],4)
		Odom_Noise.pose.pose.position.y = round(position_noisy[1],4)
		Odom_Noise.pose.pose.position.z = round(position_noisy[2],4)
		(x, y, z, w) = euler_to_quaternion(position_noisy[3], position_noisy[4], position_noisy[5]) #Transform Euler to Quaternian angles
		Odom_Noise.pose.pose.orientation.x = round(x,4)
		Odom_Noise.pose.pose.orientation.y = round(y,4)
		Odom_Noise.pose.pose.orientation.z = round(z,4)
		Odom_Noise.pose.pose.orientation.w = round(w,4)

		Odom_Noise.twist.twist.linear.x = round(velocity_noisy[0],4)
		Odom_Noise.twist.twist.linear.y = round(velocity_noisy[1],4)
		Odom_Noise.twist.twist.linear.z = round(velocity_noisy[2],4)
		Odom_Noise.twist.twist.angular.x = round(velocity_noisy[3],4)
		Odom_Noise.twist.twist.angular.y = round(velocity_noisy[4],4)
		Odom_Noise.twist.twist.angular.z = round(velocity_noisy[5],4)
		#print("pubx" + str(Odom_Noise.pose.pose.position.x))

		pub1.publish(Odom_Noise)	#Publish msg
		rate.sleep()			#Sleep with rate
		

	except rospy.ROSInterruptException:
          	pass
###################################################################################################################################################################












