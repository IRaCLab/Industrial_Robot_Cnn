#! /usr/bin/env python

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib


from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from math import pi
from math import *

import serial
from numpy import * 

import matplotlib.pyplot as plt
from math import * 
import threading

from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Point, Pose
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, MyoPose, EmgArray

 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
	       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

start_position = [-1.57, -0.351, 0, -3.14 , -1.57, -0.785]   

Q = [-1.57, -0.351, 0, -3.14 , -1.57, -0.785]                # This value will change constantly. 
						     # Q[0] is base.  Q[1] is shoulder 
						     # Q[2] is elbow. Q[3] is list 1
						     # Q[4] is list 2.Q[5] is list 3 
						     # We just use Q[0]. 
myo_data_x = 0
myo_data_y = 0


sensor_data = 450     # A box that receives sensor data
step = 0
time.sleep(1)

client = None

x, y, z = [] , [], []

moving_time = 0


def average20(list):                                ###################
	v = 0

	for m in list:
		v = float(v) + float(m)

	return v / len(list)			   #### data's average is calculated
def averageFilter(list):
	w = 0
	for n in range(-1,-31,-1):
		w = float(w) + float(list[n])
	return w / int(30)			   ###################




def callback(data):
	global myo_data_x
	global myo_data_y
	rospy.loginfo("!!!Success Myo!!!")
	myo_data_x = data.x
	myo_data_y = data.y
	 

	

	time.sleep(1)


def callback_process(data2):
	global x,y,z 
	global sensor_data
	global step 
	w = 0
	
	rospy.loginfo("!!!Success Sensor!!!")
	sensor_data = round(data2.data, 5)
	y.append(data2.data)
	if((step >= 3) and (y[-1] == y[-2] == y[-3])):
		data2.data = 800.0
		sensor_data = data2.data
	
	
	 
	
	print(sensor_data)
	
	print("sensor_data is %s ", sensor_data)
	print(step)
	

	time.sleep(1)



def listener():					# date received by sensor 


	rospy.Subscriber('/myo_raw/myo_ori', Vector3,callback, queue_size=1)
	rospy.Subscriber('chatter', Float32 ,callback_process, queue_size=1)
	rospy.spin()
	

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
		JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(0.25))]



	client.send_goal(g)
	client.wait_for_result()
    
    except KeyboardInterrupt:
	client.cancel_goal()
	raise
    except:
	raise

def stop_move_point():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
	joint_states = rospy.wait_for_message("joint_states", JointState)
	joints_pos = joint_states.position

	g.trajectory.points = [
		JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
		JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(3.0))]



	client.send_goal(g)
	client.wait_for_result()
    
    except KeyboardInterrupt:
	client.cancel_goal()
	raise
    except:
	raise
    
   
def whilemove():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    data_box_x, filter_data_box_x  = [],[]
    data_box_y, filter_data_box_y  = [],[]
    global sensor_data
    global Q
    global myo_data_x,myo_data_y,step
    global moving_time
    angle_ra_x = 0
    angle_ra_y = 0
    data_cnt=0              # Sensor data count

    base_joint = -1.57
    shoulder_joint = -0.351
    average_set = 30

    xx = 0
    xy = 0
    xx_box , xy_box = [], []

    yx = 0
    yy = 0
    yx_box , yy_box = [], []


    angle_sub_x = 0
    angle_sub_y = 0
    	
    time.sleep(3)
    print("You can move now !!! ")

    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        
	        



	while True :	
			
		data_box_x.append(myo_data_x)
		data_box_y.append(myo_data_y)
		

		
		if data_cnt > average_set:
			filter_data_box_x.append(averageFilter(data_box_x))
			filter_data_box_y.append(averageFilter(data_box_y))
		else :
			filter_data_box_x.append(average20(data_box_x))
			filter_data_box_y.append(average20(data_box_y))
		
		angle_ra_x = filter_data_box_x[-1]
		angle_ra_y = filter_data_box_y[-1]

		xx = cos(angle_ra_x)
		xy = sin(angle_ra_x)

		yx = cos(angle_ra_y)
		yy = sin(angle_ra_y)

		xx_box.append(xx)
		xy_box.append(xy)


		yx_box.append(yx)
		yy_box.append(yy)

		if(data_cnt > 2):
			if(xx > 0 and xy > 0):	          	        # x - The 1st Quadrant
				if(xx_box[-2] < xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint + angle_sub_x
				elif( xx_box[-2] > xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint - angle_sub_x
			elif(xx < 0 and xy > 0):			# x - The 2nd Quadrant
				if(xx_box[-2] < xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint + angle_sub_x
				elif( xx_box[-2] > xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint - angle_sub_x
			elif(xx < 0 and xy < 0):			# x - The 3rd Quadrant
				if(xx_box[-2] < xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint - angle_sub_x
				elif( xx_box[-2] > xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint + angle_sub_x
			elif(xx > 0 and xy < 0):			# x - The 4th Quadrant
				if(xx_box[-2] < xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint - angle_sub_x
				elif( xx_box[-2] > xx_box[-1]):
					angle_sub_x = abs(abs(filter_data_box_x[-1])-abs(filter_data_box_x[-2]))
					base_joint = base_joint + angle_sub_x


			if(yx > 0 and yy > 0):	          	        # y - The 1st Quadrant
				if(yx_box[-2] < yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint - angle_sub_y
				elif( xx_box[-2] > xx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint + angle_sub_y
			elif(yx < 0 and yy > 0):				# y - The 2nd Quadrant
				if(yx_box[-2] < yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint - angle_sub_y
				elif( yx_box[-2] > yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint + angle_sub_y
			elif(yx < 0 and yy < 0):				# y - The 3rd Quadrant
				if(yx_box[-2] < yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint + angle_sub_y
				elif( yx_box[-2] > yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint - angle_sub_y
			elif(yx > 0 and yy < 0):				# y - The 4th Quadrant
				if(yx_box[-2] < yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint + angle_sub_y
				elif( yx_box[-2] > yx_box[-1]):
					angle_sub_y = abs(abs(filter_data_box_y[-1])-abs(filter_data_box_y[-2]))
					shoulder_joint = shoulder_joint - angle_sub_y
		
		Q[0] = base_joint		 # filter_data_box[-1] is the most recent data.
		Q[1] = shoulder_joint
		

		
		

		
		
		print("base_joint : %s",base_joint)
		

		Q[0] = base_joint
		Q[1] = shoulder_joint 

		if(sensor_data < 600)	:
			while (sensor_data < 600) :
				stop_move_point()
		'''moving_time = (unit/0.00196) * 0.01

		if (moving_time > 2.0):
			moving_time = 2.0 
		elif (moving_time < 0.02 ):
			moving_time = 0.02
		print("MovingTime is %s",moving_time)'''

		move_point()



		data_cnt += 1
		step += 1

		client.send_goal(g)
		client.wait_for_result()
		time.sleep(0.5)


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
	client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
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
	    

	    th_ur.start()
	    listener()




	else:
	    print "Halting program"
	print str([Q[i]*180./pi for i in xrange(0,6)])

    except KeyboardInterrupt:
	rospy.signal_shutdown("KeyboardInterrupt")
	raise

if __name__ == '__main__': main()
