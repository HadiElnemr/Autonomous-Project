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
#####################################################################################################################################################################


####################################################################################################################################################################
### Initialize ROS Node 
rospy.init_node('Odom_Plotter', anonymous = True) #Identify Ros Node
####################################################################################################################################################################



####################################################################################################################################################################
####################################################################################################################################################################
## Method used to transform from Euler coordinates to Quaternion coordinates
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
## Method used to transform from Quaternion coordinates to Euler coordinates
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
##Create arrays for printing
t = [] #Array to hold time
X = [] #Array to hold value of x obtained by the sensor
Y = [] #Array to hold value of y obtained by the sensor
Z = [] #Array to hold value of z obtained by the sensor
Theta = [] #Array to hold value of theta obtained by the sensor

XNoisy = [] #Array to hold noisy value of x coordinates
YNoisy = [] #Array to hold noisy value of y coordinates
ZNoisy = [] #Array to hold noisy value of z coordinates
ThetaNoisy = [] #Array to hold noisy value of theta coordinates
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
	  ## (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w]) #Transform Quaternian to Euler angles
	  position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll] ##Store the position in array

	  Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)		#Round the value of x to 4 decimal places
	  Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)		#Round the value of y to 4 decimal places
	  Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)		#Round the value of z to 4 decimal places
	  Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)		#Round the value of x to 4 decimal places
	  Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)		#Round the value of y to 4 decimal places
	  Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)		#Round the value of z to 4 decimal places
	  velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]##Store the velocity in array
	  
	  
	  flag_cont = 1				#Set flag to one

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
	  ## (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w]) #Transform Quaternian to Euler angles
	  position_0 = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]##Store the position in array

	  Velocity_msg_0.linear.x = round(msg.twist.twist.linear.x, 4)		#Round the value of x to 4 decimal places
	  Velocity_msg_0.linear.y = round(msg.twist.twist.linear.y, 4)		#Round the value of y to 4 decimal places
	  Velocity_msg_0.linear.z = round(msg.twist.twist.linear.z, 4)		#Round the value of z to 4 decimal places
	  Velocity_msg_0.angular.x = round(msg.twist.twist.angular.x, 4)	#Round the value of x to 4 decimal places
	  Velocity_msg_0.angular.y = round(msg.twist.twist.angular.y, 4)	#Round the value of y to 4 decimal places
	  Velocity_msg_0.angular.z = round(msg.twist.twist.angular.z, 4)	#Round the value of z to 4 decimal places
	  velocity_0 = [Velocity_msg_0.linear.x,Velocity_msg_0.linear.y,Velocity_msg_0.linear.z,Velocity_msg_0.angular.x,Velocity_msg_0.angular.y,Velocity_msg_0.angular.z]##Store the velocity in array
	  

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
STD = [rospy.get_param("~xstd"),rospy.get_param("~ystd"),rospy.get_param("~thetastd")]
####################################################################################################################################################################
####################################################################################################################################################################
##Create method to add noise to the sensor
def noisy_add(position):
	#Define the global variables represented as the standard deviation of each state
	global position_noisy
	global XNoisy
	global YNoisy
	global ThetaNoisy

	global X
	global Y
	global Theta

	xnoise = position[0] + random.uniform(-STD[0],STD[0])		##Noisy data calculated at x
	ynoise = position[1] + random.uniform(-STD[1],STD[1])		##Noisy data calculated at y
	thetanoise = position[3] + random.uniform(-STD[2],STD[2])	##Noisy data calculated at z

	position_noisy = [xnoise, ynoise, position[2], thetanoise, position[4], position[5]]##Store the noisy position in array
 
	X.append(position[0])			#Add to the empty array of x the position of x subscribed
	Y.append(position[1])			#Add to the empty array of y the position of y subscribed
	Theta.append(position[3])		#Add to the empty array of theta the value of theta subscribed

	XNoisy.append(position_noisy[0])	#Add to the empty array of x the noisy position of x calculated
	YNoisy.append(position_noisy[1])	#Add to the empty array of y the noisy position of y calculated
	ThetaNoisy.append(position_noisy[3])	#Add to the empty array of theta the noisy value of theta calculated
	return(position_noisy)			#Return the noisy position
####################################################################################################################################################################
####################################################################################################################################################################

while 1 and not rospy.is_shutdown():
## If the subscriber subscribes a signal the method "noisy_add" should be called to add noise to the signal subscribed
	if flag_initial == 1:
		position_noisy = noisy_add(position_0)
		flag_initial = 0
	if flag_cont == 1:
		position_noisy = noisy_add(position)
		flag_cont = 0
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
##Plotting of signals from sensor and noisy signals
plt.figure(1)
line_1 = plt.plot(X, 'r-', label='X-Pos')
line_2 = plt.plot(XNoisy, 'b-', label='X-Noisy')
plt.legend()


plt.figure(2)
line_1 = plt.plot(Y, 'r-', label='Y-Pos')
line_2 = plt.plot(YNoisy, 'b-', label='Y-Noisy')
plt.legend()


plt.figure(4)
line_1 = plt.plot(Theta, 'r-', label='Theta')
line_2 = plt.plot(ThetaNoisy, 'b-', label='Theta-Noisy')
plt.legend()

plt.show(block=True)
####################################################################################################################################################################
####################################################################################################################################################################












