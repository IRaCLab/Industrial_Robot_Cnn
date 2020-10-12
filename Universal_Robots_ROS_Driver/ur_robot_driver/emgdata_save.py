#! /usr/bin/env python

import time
import rospy
import actionlib
import numpy as np
import pandas as pd

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

from std_msgs.msg import Float32
from std_msgs.msg import String
from math import pi
from math import *

import serial
from numpy import * 


from math import * 
import threading

from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Point, Pose
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, MyoPose, EmgArray

step = 0
client = None
print("Start !!! \n")
moving_time = 0


emg_box = []
emg_df = 0
count= 0
file_num = 0

def callback_process(data):
        global emg_df, file_num
        global emg_data, emg_data1, emg_data2, emg_data3, emg_data4
        global step, emg_data5, emg_data6, emg_data7, emg_data8
        global emg_box,count

		
        if( (count !=0) and ((count % 250) == 0) ):
                file_num = file_num + 1
                emg_df.to_csv('exc666_%s' % str(file_num) , index=False, encoding='cp949')
                emg_box = []  

        if( (count % 50) == 0 ):
  
                '''print("33333")
                time.sleep(1)
                print("22222")'''
                #time.sleep(1)

                print("11111")
                time.sleep(1)
                print("!GO!")
				
        rospy.loginfo("!!!Success Myo!!!  "+str(count))
	

        emg_data = data.data
        emg_data1 = emg_data[0]
        emg_data2 = emg_data[1]
        emg_data3 = emg_data[2]
        emg_data4 = emg_data[3]
        emg_data5 = emg_data[4]
        emg_data6 = emg_data[5]
        emg_data7 = emg_data[6]
        emg_data8 = emg_data[7]
	 		
        emg_box.append([emg_data1,emg_data2,emg_data3,emg_data4,emg_data5,emg_data6,emg_data7,emg_data8])
        emg_df = pd.DataFrame(emg_box, columns=['ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8'])
	
	 
	
        count = count + 1

def listener():					# date received by sensor 
	
	
        rospy.Subscriber('/myo_raw/myo_emg', EmgArray,callback_process, queue_size=10)
	
        rospy.spin()
	


def main():
   
    rospy.init_node('emgsave_node')
    # th_print = threading.Thread(target = prints)
    try:
            	 
            listener()
	    	

    except KeyboardInterrupt:
	
	
        rospy.signal_shutdown("KeyboardInterrupt")
	
        raise

if __name__ == '__main__':  

        while(1):
                main()

                break

	
	
	
