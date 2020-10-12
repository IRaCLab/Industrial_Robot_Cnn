#!/usr/bin/python

import socket

import cPickle as pickle

s1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = '127.0.0.1'

s1.bind((ip, 45000))

while True:
       data, addr = s1.recvfrom(4096)
       conv = pickle.loads(data)
       print (conv)
       break
       #print np.fromstring(conv,dtype=int)
s1.close()
