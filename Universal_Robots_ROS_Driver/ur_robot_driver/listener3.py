#!/usr/bin/env python3

import pandas as pd
import socket
import numpy as np
import pickle
import rospy
import time
import matplotlib.pyplot as plt
from ros_myo.msg import MyoArm, MyoPose, EmgArray
from std_msgs.msg import String

#Receiver ip
ip = "127.0.0.1"

#Set up socket and stuff 
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


cnt = 0 
time_data_start = 0
time_data_end = 0 

def callback(data):
    global cnt, time_data_start, time_data_end
 
    csv_test = data.data

    #csv_test = pd.read_csv('/home/sysy/files/data/1opentest1.csv')
    #Turn image into numpy-array
    if ( cnt == 0 ):
        time_data_start = time.time()


    csv_test0 = (csv_test[0]-15)/(1511-15)
    csv_test0 = round(csv_test0,5)
    csv_test1 = (csv_test[1]-15)/(1511-15)
    csv_test1 = round(csv_test1,5)
    csv_test2 = (csv_test[2]-15)/(1511-15)
    csv_test2 = round(csv_test2,5)
    csv_test3 = (csv_test[3]-15)/(1511-15)
    csv_test3 = round(csv_test3,5)

    csv_test4 = (csv_test[4]-15)/(1511-15)
    csv_test4 = round(csv_test4,5)
    csv_test5 = (csv_test[5]-15)/(1511-15)
    csv_test5 = round(csv_test5,5)
    csv_test6 = (csv_test[6]-15)/(1511-15)
    csv_test6 = round(csv_test6,5)
    csv_test7 = (csv_test[7]-15)/(1511-15)
    csv_test7 = round(csv_test7,5)

    test_data = [csv_test0,csv_test1,csv_test2,csv_test3,csv_test4,csv_test5,csv_test6,csv_test7]
    arr = np.array(test_data)
    print(arr)
    

    
    #Encode each array
    msg = pickle.dumps(arr,protocol=2)

    #Send msg to ip with port
    s.sendto(msg, (ip,45000))
    
    cnt = cnt + 1
   
   

    if ( cnt % 50 == 0):
        time_data_end = time.time()
        time_sub = round(time_data_end - time_data_start,4)
        print("time sub : ",time_sub)
        #time.sleep(2)
        cnt = 0

    '''if(each %50 == 49):
        time.sleep(3)'''

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/myo_raw/myo_emg", EmgArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()
    
