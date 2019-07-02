#!/usr/bin/env python

import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
data = sys.argv[3]

print("port = " + str(port))

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
print('Client sending aiwc.get_info')
s.sendall('aiwc.get_info'.encode('utf-8'))
data = s.recv(1024)
print('Client received data: ', data.decode('utf-8'))
print('Client sending aiwc.ready')
s.sendall('aiwc.ready'.encode('utf-8'))
data = s.recv(1024)
print('Client received data: ', data.decode('utf-8'))
