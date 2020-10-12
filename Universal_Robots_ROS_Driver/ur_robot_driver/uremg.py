#! /usr/bin/env python

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import math


from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from math import pi
from math import *

import serial
from numpy import * 


from math import * 
import threading

from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Point, Pose
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, MyoPose, EmgArray

from scipy import io
import random
from numpy import argmax

import numpy as np

import socket
import pickle











s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = '127.0.0.1'
s2.bind((ip, 44002)) 



 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
	       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

start_position = [-0.785, -0.351, 0, -1.279326 , -1.57, -1.57]   

Q = [-0.785,-0.351, 0, -1.279326 , -1.57, -1.57]                # This value will change constantly. 
						    	     # Q[0] is base.  Q[1] is shoulder 
						   	     # Q[2] is elbow. Q[3] is list 1
							     # Q[4] is list 2.Q[5] is list 3 
						             # We use Q[0] and Q[1]. 
myo_data_x = 0
myo_data_y = 0


data_s = []
step = 0
time.sleep(1)

client = None

x, y, z = [] , [], []

moving_time = 0







print("Start !!! \n")



def start_point(): 
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=start_position, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]



        client.send_goal(g)
        client.wait_for_result()
    
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise




def move_point():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]     # 0.01s is the minimum unit time to move the robot.



        client.send_goal(g)
        client.wait_for_result()
    
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise


   
def whilemove():
    global joints_pos, result
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    
    
    global Q
    global myo_data_x, myo_data_y, step
    global moving_time
    global data_s
    angle_ra_x = 0
    angle_ra_y = 0
    data_cnt=0              # Sensor data count

    base_joint = -1.57         
    shoulder_joint = -0.351 

    Q[0] = base_joint
    Q[1] = shoulder_joint

    time.sleep(3)
    print("You can move now !!! ")
    
    try:

            while True:
                data, addr = s2.recvfrom(4096)
                result = pickle.loads(data)
	        
                if (result == 'right'):
                    Q[0] = -1.57	
                    move_point() 
                elif(result == 'left'):
                    Q[0] = 0
                    move_point() 
                elif(result == '4finger'):
                    Q[1] = Q[1] - 0.02

		

                print("base_joint : %s",Q[0])

		              


                client.send_goal(g)
                client.wait_for_result()
                time.sleep(0.02)


    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise




def main():
    global client
    th_ur = threading.Thread(target=whilemove)
    
    try:

        rospy.init_node("UR3_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
	    prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
	    for i, name in enumerate(JOINT_NAMES):
		JOINT_NAMES[i] = prefix + name
	print "This program makes the robot move between the following three poses:"
	print str([Q[i]*180./pi for i in xrange(0,6)])

	print "Please make sure that your robot can move freely between these poses before proceeding!"
	inp = raw_input("Continue? y/n: ")[0]
	if (inp == 'y'):
	    
	    start_point()
	    

	    #th_ur.start()
	    
            
            whilemove()




	else:
	    print "Halting program"
	print str([Q[i]*180./pi for i in xrange(0,6)])

    except KeyboardInterrupt:
	rospy.signal_shutdown("KeyboardInterrupt")
	raise

if __name__ == '__main__': 
	main()
