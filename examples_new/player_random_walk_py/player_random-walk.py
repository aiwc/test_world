#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

from __future__ import print_function

import random
import socket
import sys
import time

import base64
import numpy as np

# reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4
DEADLOCK = 5
GOALKICK = 6
CORNERKICK = 7
PENALTYKICK = 8
HALFTIME = 9
EPISODE_END = 10

# game_state
STATE_DEFAULT = 0
STATE_KICKOFF = 1
STATE_GOALKICK = 2
STATE_CORNERKICK = 3
STATE_PENALTYKICK = 4

# coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2
ACTIVE = 3
TOUCH = 4


class Received_Image(object):
    def __init__(self, resolution, colorChannels):
        self.resolution = resolution
        self.colorChannels = colorChannels
        # need to initialize the matrix at timestep 0
        self.ImageBuffer = np.zeros((resolution[1], resolution[0], colorChannels))  # rows, columns, colorchannels

    def update_image(self, received_parts):
        self.received_parts = received_parts
        for i in range(0, len(received_parts)):
            dec_msg = base64.b64decode(self.received_parts[i].b64, '-_')  # decode the base64 message
            np_msg = np.fromstring(dec_msg, dtype=np.uint8)  # convert byte array to numpy array
            reshaped_msg = np_msg.reshape((self.received_parts[i].height, self.received_parts[i].width, 3))
            for j in range(0, self.received_parts[i].height):  # y axis
                for k in range(0, self.received_parts[i].width):  # x axis
                    self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 0] = reshaped_msg[j, k, 0]  # blue
                    self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 1] = reshaped_msg[j, k, 1]  # green
                    self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 2] = reshaped_msg[j, k, 2]  # red


class SubImage(object):
    def __init__(self, x, y, width, height, b64):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.b64 = b64


class Frame(object):
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.subimages = None
        self.coordinates = None
        self.half_passed = None


class Session():
    """
    AI Base + Random Walk
    """

    def __init__(self):
        self.host = sys.argv[1]
        self.port = int(sys.argv[2])
        self.key = sys.argv[3]
        self.data = sys.argv[4]
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def send(self, message, arguments=[]):
        message = 'aiwc.' + message + '("%s"' % self.key
        for argument in arguments:
            if isinstance(argument, str):  # string
                message += ', "%s"' % argument
            else:  # number
                message += ', %s' % argument
        message += ')'
        self.socket.sendall(message.encode())

    def receive(self):
        data = self.socket.recv(1024).decode()
        return data

    def init_variables(self, info):
        # Here you have the information of the game (virtual init() in random_walk.cpp)
        # List: game_time, number_of_robots
        #       field, goal, penalty_area, goal_area, resolution Dimension: [x, y]
        #       ball_radius, ball_mass,
        #       robot_size, robot_height, axle_length, robot_body_mass, ID: [0, 1, 2, 3, 4]
        #       wheel_radius, wheel_mass, ID: [0, 1, 2, 3, 4]
        #       max_linear_velocity, max_torque, codewords, ID: [0, 1, 2, 3, 4]
        # self.game_time = info['game_time']
        self.number_of_robots = info['number_of_robots']

        # self.field = info['field']
        # self.goal = info['goal']
        # self.penalty_area = info['penalty_area']
        # self.goal_area = info['goal_area']
        self.resolution = info['resolution']

        # self.ball_radius = info['ball_radius']
        # self.ball_mass = info['ball_mass']

        # self.robot_size = info['robot_size']
        # self.robot_height = info['robot_height']
        # self.axle_length = info['axle_length']
        # self.robot_body_mass = info['robot_body_mass']

        # self.wheel_radius = info['wheel_radius']
        # self.wheel_mass = info['wheel_mass']

        self.max_linear_velocity = info['max_linear_velocity']
        # self.max_torque = info['max_torque']
        # self.codewords = info['codewords']

        self.colorChannels = 3
        self.end_of_frame = False
        self.image = Received_Image(self.resolution, self.colorChannels)
        return

    def start(self):
        self.send('get_info')
        info = self.receive()
        self.init_variables(info)
        self.send('ready')
        self.receive()

    def set_wheel(self, robot_wheels):
        self.send('set_speed', robot_wheels)

    def receive_frame(self, f):
        # initiate empty frame
        received_frame = Frame()
        received_subimages = []

        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'half_passed' in f:
            received_frame.half_passed = f['half_passed']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
            # Comment the next lines if you don't need to use the image information
            for s in received_frame.subimages:
                received_subimages.append(SubImage(s['x'],
                                                   s['y'],
                                                   s['w'],
                                                   s['h'],
                                                   s['base64'].encode('utf8')))
            self.image.update_image(received_subimages)
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']
        if 'EOF' in f:
            self.end_of_frame = f['EOF']

        # self.printConsole(received_frame.time)
        # self.printConsole(received_frame.score)
        # self.printConsole(received_frame.reset_reason)
        # self.printConsole(self.end_of_frame)

        if (self.end_of_frame):
            # To get the image at the end of each frame use the variable:
            # self.image.ImageBuffer

            # (virtual update() in random_walk.cpp)
            wheels = []
            for i in range(self.number_of_robots):
                wheels.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
                wheels.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
            self.set_wheel(wheels)

            if received_frame.reset_reason == GAME_END:
                # (virtual finish() in random_walk.cpp)
                # save your data
                with open(self.data + '/result.txt', 'w') as output:
                    # output.write('yourvariables')
                    output.close()
            self.end_of_frame = False


if __name__ == '__main__':

    try:
        unicode
    except NameError:
        # Define 'unicode' for Python 3
        def unicode(s, *_):
            return s

    def to_unicode(s):
        return unicode(s, "utf-8")

    # create a TCP/IP session object
    session = Session()
    session.start()
    while True:
        if session.spin(0.05):  # run for 50 ms
            break
