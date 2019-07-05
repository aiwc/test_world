#!/usr/bin/env python

import random
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
key = sys.argv[3]
data = sys.argv[4]


def send(message, arguments=[]):
    message = 'aiwc.' + message + '("%s"' % key
    for argument in arguments:
        if isinstance(argument, str):  # string
            message += ', "%s"' % argument
        else:  # number
            message += ', %s' % argument
    message += ')'
    s.sendall(message.encode())


def receive():
    data = s.recv(1024)
    return data.decode()


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
print('Client sending aiwc.get_info')
send('get_info')
r = receive()
print('Client received data: %s' % r)
print('Client sending aiwc.ready')
send('ready')
while True:
    r = receive()
    print('Client received data, ball coordinates: %s' % r)
    send('set_speeds', [random.uniform(-2, 2), random.uniform(-2, 2), 1, 1, 1, -1, -1, 1, -1, -1])
