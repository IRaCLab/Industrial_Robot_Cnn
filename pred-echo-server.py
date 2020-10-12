#!/usr/bin/python
import tensorflow as tf
from tensorflow import keras
from scipy import io
import random
from numpy import argmax

import numpy as np

import socket
import pickle
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

s1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ip = '127.0.0.1'
s.bind((ip, 45000)) 

emg_box = []
cnt = 0

while True:
    
    data, addr = s.recvfrom(4096)
    data2= pickle.loads(data)
    
    emg_data = np.array(data2)
     
    emg_data1 = emg_data[0]
    emg_data2 = emg_data[1]
    emg_data3 = emg_data[2]
    emg_data4 = emg_data[3]
    emg_data5 = emg_data[4]
    emg_data6 = emg_data[5]    
    emg_data7 = emg_data[6]  
    emg_data8 = emg_data[7] 
    emg_box.append([emg_data1,emg_data2,emg_data3,emg_data4,
                 
                    
                    emg_data5,emg_data6,emg_data7,emg_data8])
    cnt = cnt + 1
    #print(cnt)
    if ( cnt % 50 == 0):
        # print (emg_box)
        emg_box = np.reshape(emg_box,(1,50,8))
        X_test = emg_box*emg_box
        
        test_num=0
        X_test = X_test[..., np.newaxis]
        
        # 2. 

        model = tf.keras.models.load_model('Gesture_Classifier.h5')

        # 3. 
        yhat = model.predict_classes(X_test)

        pred = yhat[test_num]
        print("result : ", pred)
        emg_box = []
        if(pred==0):
            char = 'close'
            msg = pickle.dumps(char, protocol=2)
            s1.sendto(msg, (ip,44001))
           
        
               
        elif(pred==1):
            char = 'open'
            msg = pickle.dumps(char, protocol=2)
            s1.sendto(msg, (ip,44001))
        elif(pred==2) : 
            char = 'rest'
            '''msg = pickle.dumps(char, protocol=2)
            s1.sendto(msg, (ip,44001))
            msg = pickle.dumps(char, protocol=2)
            s2.sendto(msg, (ip,44002))'''
        else :
            if(pred==3):
                char = 'right'
                msg = pickle.dumps(char, protocol=2)
                s2.sendto(msg, (ip,44002))
            elif(pred==4):
                char = 'left'
                msg = pickle.dumps(char, protocol=2)
                s2.sendto(msg, (ip,44002))
            
            
                
            
                
        time.sleep(0.5)
        if(cnt == 10000):
            break
    
                  #print np.fromstring(conv,dtype=int)
s.close()
