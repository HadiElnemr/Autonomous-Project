#! /usr/bin/env python 
# A basic python code to subscribe from the turtlebot its position and orientation through odometery sensor

### Libraries to be imported
import rospy #library to connect ROS with python
import matplotlib.pyplot as plt #library used for plotting 
import numpy as np #library used for mathematical operations
import math
import time
import tf #library used for states transformation from Quaternian to Euler and vice versa

from nav_msgs.msg import Odometry #import msg data type "Odometry" from nav_msgs dependency to be subscribed 
from geometry_msgs.msg import Twist, Pose #import msg data type "Twist" and "Pose" from geometry_msgs dependency to be published and subscribed
#####################################################################################################################################################################


####################################################################################################################################################################
### Initialize ROS Node 
rospy.init_node('Kalman_Filter', anonymous = True) #Identify Ros Node
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
## ROS Publisher Code
pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Identify the publisher "pub1" to publish on topic "/Odom_Values" to send message of type "Odometry"
Control_Input = Twist() #Identify msg variable of data type Odometry
rate = rospy.Rate(10) # rate of publishing msg 10hz
####################################################################################################################################################################
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
XModel = [] #Array to hold value of x obtained by the sensor
YModel = [] #Array to hold value of y obtained by the sensor
ThetaModel = [] #Array to hold value of theta obtained by the sensor

XNoisy = [] #Array to hold noisy value of x coordinates
YNoisy = [] #Array to hold noisy value of y coordinates
ThetaNoisy = [] #Array to hold noisy value of theta coordinates

XFiltered = [] #Array to hold Filtered value of x coordinates
YFiltered = [] #Array to hold Filtered value of y coordinates
ThetaFiltered = [] #Array to hold Filtered value of theta coordinates

TimeTotal = []
####################################################################################################################################################################



####################################################################################################################################################################
####################################################################################################################################################################
#ROS Subscriber Code for Position
flag_cont = 0			#Initialize flag by zero
pos_msg = Pose()		#Identify msg variable of data type Pose
position = np.zeros((1,6))	#Identify array of six elements all initialized by zero
Velocity_msg = Twist()		#Identify msg variable of data type Twist
velocity = np.zeros((1,6))	#Identify array of six elements all initialized by zero
Time = 0
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
	  global Time

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
	  
	  Time = msg.header.stamp.to_sec() 	#Extract the time of the simulation
	  flag_cont = 1				#Set flag to one

sub2 = rospy.Subscriber('/Odom_Values', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	      #Identify msg variable of data type Pose
position_0 = np.zeros((1,6))  #Identify array of six elements all initialized by zero
flag_initial = 0	      #Initialize flag by zero
Velocity_msg_0 = Twist()      #Identify msg variable of data type Pose
velocity_0 = np.zeros((1,6))  #Identify array of six elements all initialized by zero
Time_0 = 0
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
	  global Time_0

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
	  
	  Time_0 = msg.header.stamp.to_sec()

	  flag_initial = 1			#Set the flag to one
	  sub1.unregister()			#Unsubsribe from this topic

sub1 = rospy.Subscriber('/Odom_Values', Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
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
## Prediction stage in Kalman filter
def kf_prediction(Xprev,Pprev, A, Q, B, U):
    Xpredicted = np.matmul(A, Xprev) + np.dot(B, U)			##Predicted state vector			
    Ppredicted = np.matmul(A, np.matmul(Pprev, np.transpose(A))) + Q	##Predicted error co-variance matrix	
    return (Xpredicted, Ppredicted)
####################################################################################################################################################################
####################################################################################################################################################################


####################################################################################################################################################################
####################################################################################################################################################################
## Correction stage in Kalman filter
def kf_correction(Xpredicted, Ppredicted, C, Z, R):			
    CTrans = np.transpose(C)				
    num = np.matmul(Ppredicted, CTrans)		##Numerature of Kalman gain equation
    den1 = np.matmul(C, Ppredicted) 		##CMatrix * PMatrix
    den2 = np.matmul(den1, CTrans) + R  	##CMatrix * PMatrix * CMatrix^T _+ R
    den = np.matrix(den2)  			##Place denemrature in matrix form  
    deninv = den.getI()				##(CMatrix * PMatrix * CMatrix^T _+ R) Inverse 	
    KGain = np.matmul(num, deninv) 		##Calculate the Kalman gain
    print("KG" + str(KGain))

    Xfiltered = Xpredicted + np.matmul(KGain, (Z - np.matmul(C, Xpredicted))) 	##Estimated updated state vector
    Pfiltered = Ppredicted - np.matmul(KGain, np.matmul(C, Ppredicted)) 	##Estimated updated error co-variance matrix
    return (Xfiltered, Pfiltered)
####################################################################################################################################################################

####################################################################################################################################################################
####################################################################################################################################################################
##Initialization:
P = [[100,0,0],[0,100,0],[0,0,1]]		##Error co-variance matrix P (3x3)
Q = [[0.01,0,0],[0,0.01,0],[0,0,0.01]]		##Process noise matrix Q (3x3)
R = 0.1						##Measurement noise matrix R (1x1)

U = [[0.2],[0]]					##Control input matrix U (2x1)
X = [[0],[0],[0]]				##Initial state X (3x1)

A = [[1,0,0],[0,1,0],[0,0,1]]			##State transition matrix A (3x3)
B = [[0.1,0],[0,0],[0,0.1]]			##Input matrix B (3x2)
C = [[1,0,0]]					##Measurement matrix C (1x3)

iterations = 50
i = 0
####################################################################################################################################################################

while 1 and not rospy.is_shutdown():
	if i < iterations:
		if flag_initial == 1:
			##Get the initial states
			x_p =0 								
			y_p = 0
			theta_p = 0
		        
                        ##Get the time in seconds
			t = Time_0

			X = [[x_p],[y_p],[theta_p]] 	##Set the states of the system
			Z = x_p 			##Set the sensor reading for the x position of the robot

			(Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
			(Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z , R)	##Get the corrected states
			flag_initial = 0

		if flag_cont == 1:
			##Get the initial states
			x_p = position[0]
			y_p = position[1]
			theta_p = position[3]

			Z = x_p ##Set the sensor reading for the x position of the robot

			t = Time	##Get the time in seconds

			X = Xfiltered	##Update the states with the filtered states
			P = Pfiltered	##Update the error co-variance matrix

			(Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
			(Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z, R)		##Get the corrected states
			flag_cont = 0
	
		XModel.append(Xpredicted.item(0)) 	#Array to hold value of x obtained by the sensor
		YModel.append(Xpredicted.item(1)) 	#Array to hold value of y obtained by the sensor
		ThetaModel.append(Xpredicted.item(2)) 	#Array to hold value of theta obtained by the sensor
        	
		XNoisy.append(x_p) 			#Array to hold noisy value of x coordinates
		YNoisy.append(y_p) 			#Array to hold noisy value of y coordinates
		ThetaNoisy.append(theta_p)		#Array to hold noisy value of theta coordinates

		XFiltered.append(Xfiltered.item(0)) 	#Array to hold Filtered value of x coordinates
		YFiltered.append(Xfiltered.item(1))	#Array to hold Filtered value of y coordinates
		ThetaFiltered.append(Xfiltered.item(2)) #Array to hold Filtered value of theta coordinates

		TimeTotal.append(t)

		Control_Input.linear.x = 0.2
		Control_Input.linear.y = 0
		Control_Input.linear.z = 0
		Control_Input.angular.x = 0
		Control_Input.angular.y = 0
		Control_Input.angular.z = 0
		pub1.publish(Control_Input)
		rate.sleep()

	else:
		Control_Input.linear.x  = 0
		Control_Input.angular.z = 0
		pub1.publish(Control_Input)
		rospy.signal_shutdown("for plotting")
	i=i+1
        
#####################################################################################################################################################################
#####################################################################################################################################################################
##Plotting of signals from sensor and noisy signals
plt.figure(1)
line_1 = plt.plot(XModel, 'r-', label='X-Model')
line_2 = plt.plot(XNoisy, 'b-', label='X-Noisy')
line_3 = plt.plot(XFiltered, 'g-', label='X-Filtered')
plt.legend()


plt.figure(2)
line_1 = plt.plot(YModel, 'r-', label='Y-Model')
line_2 = plt.plot(YNoisy, 'b-', label='Y-Noisy')
line_3 = plt.plot(YFiltered, 'g-', label='Y-Filtered')
plt.legend()

plt.figure(3)
line_1 = plt.plot(ThetaModel, 'r-', label='Theta-Model')
line_2 = plt.plot(ThetaNoisy, 'b-', label='Theta-Noisy')
line_3 = plt.plot(ThetaFiltered, 'g-', label='Theta-Filtered')
plt.legend()


plt.show(block=True)
####################################################################################################################################################################
####################################################################################################################################################################

