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
import random
from matplotlib import pyplot as plt

time.sleep(5)

#########################################################################################################
## ROS Publisher Code
# pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Identify the publisher "pub1" to publish on topic "/Odom_Values" to send message of type "Odometry"
# # Control_Input = Twist() #Identify msg variable of data type Odometry

#########################################################################################################

##Initialization:
P = [[100,0,0],[0,100,0],[0,0,1]]		##Error co-variance matrix P (3x3)
Q = [[0.01,0,0],[0,0.01,0],[0,0,0.01]]		##Process noise matrix Q (3x3)
R = 0.1						##Measurement noise matrix R (1x1)

U = [[0.2],[0]]					##Control input matrix U (2x1)
X = [[0],[0],[0]]				##Initial state X (3x1)

A = [[1,0,0],[0,1,0],[0,0,1]]			##State transition matrix A (3x3)
B = [[0.1,0],[0,0],[0,0.1]]			##Input matrix B (3x2)
C = [[1,0,0]]					##Measurement matrix C (1x3)
#########################################################################################################

#Initialize ROS Node
rospy.init_node('Localization_Module') #Identify ROS Node

def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]

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

def add_noise(position, velocity):
    #Define the global variables represented as the standard deviation of each state
    # global position_noisy
    # global XNoisy
    # global YNoisy
    # global ZNoisy
    # global ThetaNoisy
    # global count
    error_std = rospy.get_param("~error_std")
    xnoise = position.position.x + random.gauss(0,error_std)			##Noisy data calculated at x
    ynoise = position.position.y + random.gauss(0,error_std)			##Noisy data calculated at y
    # thetanoise = position.orientation.z + random.gauss(0,error_std)		##Noisy data calculated at z
    position_noisy = Pose()
    position_noisy.position.x = xnoise
    position_noisy.position.y = ynoise
    position_noisy.position.z = position.position.z
    position_noisy.orientation.x = position.orientation.x
    position_noisy.orientation.y = position.orientation.y
    position_noisy.orientation.z = position.orientation.z
    position_noisy.orientation.w = position.orientation.w
    
    return position_noisy, velocity			#Return the noisy position and the velocity

def kf_prediction(Xprev,Pprev, A, Q, B, U):
    Xpredicted = np.matmul(A, Xprev) + np.dot(B, U)			##Predicted state vector			
    Ppredicted = np.matmul(A, np.matmul(Pprev, np.transpose(A))) + Q	##Predicted error co-variance matrix	
    return (Xpredicted, Ppredicted)

def kf_correction(Xpredicted, Ppredicted, C, Z, R):			
    CTrans = np.transpose(C)				
    num = np.matmul(Ppredicted, CTrans)		##Numerature of Kalman gain equation
    den1 = np.matmul(C, Ppredicted) 		##CMatrix * PMatrix
    den2 = np.matmul(den1, CTrans) + R  	##CMatrix * PMatrix * CMatrix^T _+ R
    den = np.matrix(den2)  			##Place denemrature in matrix form  
    deninv = den.getI()				##(CMatrix * PMatrix * CMatrix^T _+ R) Inverse 	
    KGain = np.matmul(num, deninv) 		##Calculate the Kalman gain
    # print("KG" + str(KGain))

    Xfiltered = Xpredicted + np.matmul(KGain, (Z - np.matmul(C, Xpredicted))) 	##Estimated updated state vector
    Pfiltered = Ppredicted - np.matmul(KGain, np.matmul(C, Ppredicted)) 	##Estimated updated error co-variance matrix
    return (Xfiltered, Pfiltered)

def callback_loc(data):
    global Actual_position,Actual_Speed	#Identify msg variable created as global variable
    Actual_position = data.pose[2]
    Actual_Speed = data.twist[2]

def update_line(hl, new_data):
    hl.set_xdata(np.append(hl.get_xdata(), new_data))
    hl.set_ydata(np.append(hl.get_ydata(), new_data))
    plt.draw()


if __name__ == '__main__':
    ########################################################################
    x=0
    y=0
    v=0
    theta=0

    actual_positions_x = []
    actual_positions_y = []
    
    filtered_positions_x = []
    filtered_positions_y = []
    
    noisy_positions_x = []
    noisy_positions_y = []
    
    predicted_positions_x = []
    predicted_positions_y = []

    hl, = plt.plot([], [])

    iterations = 50
    i = 0

    pub1 = rospy.Publisher('/Act_pose', Pose, queue_size=10)
    pub2 = rospy.Publisher('/Act_speed', Twist, queue_size=10)
    
    position = None
    speed = None
    Actual_position = None
    Actual_Speed = None
    
    rate = rospy.Rate(10) # 10 Hz
    
    sub = rospy.Subscriber('/steer_bot/gazebo/model_states', ModelStates, callback_loc)
    
    ########################################################################

    ##Get the initial states
    while Actual_position is None:
        x_p =0							
        y_p = 0
        theta_p = 0
        Pfiltered = None

        X = [[x_p],[y_p],[theta_p]] 	##Set the states of the system
        Z = x_p 			##Set the sensor reading for the x position of the robot

        (Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
        (Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z , R)	##Get the corrected states 
        i = 0

    ########################################################################

    while 1 and not rospy.is_shutdown():
        position = Actual_position
        speed = Actual_Speed
        # if position is None:
        #     continue
        position, speed = add_noise(position, speed)

        X = [[x_p],[y_p],[theta_p]] 	##Set the states of the system
        Z = x_p 			##Set the sensor reading for the x position of the robot

        (Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
        (Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z , R)	##Get the corrected states
        
        ##Get the states
        # x_p = 0 if position is None else position.position.x
        # y_p = 0 if position is None else position.position.y
        # theta_p = 0 if position is None else quaternion_to_euler(position.orientation.x, position.orientation.y,
        #                                                           position.orientation.z, position.orientation.w)[0]

        x_p = position.position.x
        y_p = position.position.y
        theta_p = quaternion_to_euler(position.orientation.x, position.orientation.y,
                                                                  position.orientation.z, position.orientation.w)[0]
        
        # print("position x:", x_p)
        # print("position y:", y_p)
        # print("position theta:", theta_p)

        X = [[x_p],[y_p],[theta_p]] 	##Set the states of the system
        Z = x_p ##Set the sensor reading for the x position of the robot


        X = Xfiltered	##Update the states with the filtered states
        P = P if Pfiltered is None else Pfiltered	##Update the error co-variance matrix

        (Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
        (Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z, R)		##Get the corrected states
        pose_filtered = Pose()
        pose_filtered.position.x = Xfiltered[0]
        pose_filtered.position.y = Xfiltered[1]
        
        pose_quaternion = euler_to_quaternion(Xfiltered[2], 0, 0)

        pose_filtered.orientation.x, pose_filtered.orientation.y,\
        pose_filtered.orientation.z, pose_filtered.orientation.w = pose_quaternion[0], pose_quaternion[1],\
                                                                   pose_quaternion[2], pose_quaternion[3]
        
        actual_positions_x.append(Actual_position.position.x)
        actual_positions_y.append(Actual_position.position.y)
        
        filtered_positions_x.append(float(pose_filtered.position.x))
        filtered_positions_y.append(float(pose_filtered.position.y))
        
        noisy_positions_x.append(position.position.x)
        noisy_positions_y.append(position.position.y)
        
        predicted_positions_x.append(float(Xpredicted[0]))
        predicted_positions_y.append(float(Xpredicted[1]))

        pub1.publish(pose_filtered)
        
        if speed is not None:
            pub2.publish(speed)

        # update_line(hl,actual_positions_x[-1])
        i += 1
        print("iter:", i)
        rate.sleep()
        if i % 180 == 0:    
            plt.plot(actual_positions_x, 'g', label='actual positions in x')
            plt.plot(filtered_positions_x, 'r', label='filtered positions in x')
            plt.show()
            plt.plot(actual_positions_y, 'g', label='actual positions in y')
            plt.plot(filtered_positions_y, 'r', label='filtered positions in y')
            plt.show()
            plt.plot(actual_positions_y, actual_positions_x, 'g', label='actual positions')
            plt.plot(noisy_positions_y, noisy_positions_x, 'r', label='noisy positions') 
            plt.show()
            plt.plot(actual_positions_y, actual_positions_x, 'g', label='actual positions')
            plt.plot(predicted_positions_y, predicted_positions_x, 'r', label='predicted positions')
            plt.show()
            