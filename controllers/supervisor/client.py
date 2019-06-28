#!/usr/bin/env python

import socket

host = 'localhost'
port = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
print('Sending HELLO')
s.sendall('HELLO'.encode('utf-8'))
data = s.recv(1024)
print('received data: ', data.decode('utf-8'))
print('Sending HELLO WORLD')
s.sendall('HELLO WORLD'.encode('utf-8'))
data = s.recv(1024)
print('received data: ', data.decode('utf-8'))
