#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
# from sensor_msgs.msg import JointState
# from tf.transformations import euler_from_quaternion
import math
import time
from matplotlib import pyplot as plt

def QtoE(x,y,z,w):
    t0=+2.0*(w*z+x*y)
    t1=+1.0-2.0*(y*y+z*z)
    yaw=math.atan2(t0,t1)
    return yaw

def callback(data, args):
    pub = args[0]
    lane_y_des = args[1]
    y_actuals = args[2]
    times = args[3]
    start = args[4]

    x_centre_curr = data.pose[1].position.x
    y_centre_curr = data.pose[1].position.y

    # rospy.loginfo(" (Lateral Control node): x_centre_curr: " + str(x_centre_curr) + "    y_centre_curr: " + str(y_centre_curr) + '\n\n')
   
    # orientation_q = data.pose[1].pose[1].orientation
    orientation_e = QtoE(data.pose[1].orientation.x, data.pose[1].orientation.y,
                         data.pose[1].orientation.z, data.pose[1].orientation.w)
    
    # get front axle position
    L = 0.5 # Length of base
    x_front_curr = x_centre_curr + L/2 * math.cos(orientation_e)
    y_front_curr = y_centre_curr + L/2 * math.sin(orientation_e)

    # theta_error: Heading angular error 
    y_err = lane_y_des - y_front_curr
    heading_des = 0
    theta_error = heading_des - orientation_e #math.atan2(y_err,x_err)
    theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))


    # d_f: lateral distance error measured from the centre of the front axle to the nearest point on the path
    # d_error = math.sqrt(x_err**2 + y_err**2)
    d_f = y_err

    k_v = 0.5 # constant
    # k_v = 1 # constant
    
    v = data.twist[1].linear.x    #odom_velocity_x

    ##### Stanley Controller #####
    delta = theta_error + math.atan2(k_v * d_f, v)  
    delta = math.copysign(min(abs(delta), 35/180*math.pi), delta)



    # Twist msg
    twist_msg = Twist()
    # twist_msg.linear.x = 5 #0.3 * d_error # if d_f > 1 else 0
    # twist_msg.linear.y = 0.0 
    # twist_msg.linear.z = 0.0 
    # twist_msg.angular.x = 0.0 
    # twist_msg.angular.y = 0.0 
    twist_msg.angular.z = delta

    pub.publish(twist_msg)

    #### Evaluation ####
    times.append(rospy.get_rostime().secs - start.secs)
    print(y_err, rospy.get_rostime().secs - start.secs)
    y_actuals.append(y_err)
    

    
    
def main():
    rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_18')
    # global y_actuals
    y_actuals = []
    times = []
    
    lane_y_des = 2
    if rospy.has_param('/steer_bot/Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_18/lane_y_des'):
        
        lane_y_des = rospy.get_param('/steer_bot/Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_18/lane_y_des')
    
    # pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('/lat_control', Twist, queue_size=10)
    
    time.sleep(5)
    start = rospy.get_rostime()
    rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, callback, (pub,lane_y_des, y_actuals, times, start))
    
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        print(len(y_actuals))
        # if len(y_actuals) > 100:
        #     plt.plot(times, y_actuals)
        #     plt.ylabel('Lateral distance error')
        #     # plt.legend(['Actual Velocity', 'Desired Velocity'])
        #     plt.savefig('/home/hadi/catkin_ws/src/Autonomous_Systems_Project_Team_18/images/lateral_error_3.png')
        #     plt.show()
        rate.sleep()

if __name__ == '__main__':
    main()
