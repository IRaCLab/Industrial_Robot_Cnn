#!/usr/bin/env python3

import pandas as pd
import socket
import numpy as np
import pickle

import time

#Upload image
#csv_test = pd.read_csv('/home/sysy/files/data/1opentest1.csv')
#Turn image into numpy-array

#csv_test = (csv_test-15)/(1511-15)
#csv_test = round(csv_test,5)

arr = [1,2,3,4,5]



#Receiver ip
ip = "127.0.0.1"

#Set up socket and stuff 
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
arr = 'Hello World!'

#Loop through each array (5 for test)
while True:
       

       #Encode each array
       msg = pickle.dumps(arr,protocol=2)

       #Send msg to ip with port
       s.sendto(msg, (ip,50000))
       time.sleep(0.02)

       '''if(each %50 == 49):
           time.sleep(3)'''

s.close()
