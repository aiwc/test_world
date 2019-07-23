#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import base64
import json
import numpy as np
import random
import socket
import sys
import time


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


class ReceivedImage(object):
    def __init__(self, resolution, color_channels):
        self.resolution = resolution
        self.color_channels = color_channels
        # need to initialize the matrix at timestep 0
        self.image_buffer = np.zeros((resolution[1], resolution[0], color_channels))  # rows, columns, color channels

    def update_image(self, received_parts):
        self.received_parts = received_parts
        for i in range(0, len(received_parts)):
            dec_msg = base64.b64decode(self.received_parts[i].b64, '-_')  # decode the base64 message
            np_msg = np.fromstring(dec_msg, dtype=np.uint8)  # convert byte array to numpy array
            reshaped_msg = np_msg.reshape((self.received_parts[i].height, self.received_parts[i].width, 3))
            for j in range(0, self.received_parts[i].height):  # y axis
                for k in range(0, self.received_parts[i].width):  # x axis
                    self.image_buffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 0] = reshaped_msg[j, k, 0]  # b
                    self.image_buffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 1] = reshaped_msg[j, k, 1]  # g
                    self.image_buffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 2] = reshaped_msg[j, k, 2]  # r


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


class Player():
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
        data = self.socket.recv(1024)
        return data.decode()

    def set_speeds(self, speeds):
        self.send('set_speeds', speeds)

    def get_info(self, info):  # you should override this method
        print("get_info() method called", info)
        # Here you have the information of the game (virtual init() in random_walk.cpp)
        # List: game_time, number_of_robots
        #       field, goal, penalty_area, goal_area, resolution Dimension: [x, y]
        #       ball_radius, ball_mass,
        #       robot_size, robot_height, axle_length, robot_body_mass, ID: [0, 1, 2, 3, 4]
        #       wheel_radius, wheel_mass, ID: [0, 1, 2, 3, 4]
        #       max_linear_velocity, max_torque, codewords, ID: [0, 1, 2, 3, 4]
        # self.game_time = info['game_time']
        # self.number_of_robots = info['number_of_robots']
        # self.field = info['field']
        # self.goal = info['goal']
        # self.penalty_area = info['penalty_area']
        # self.goal_area = info['goal_area']
        # self.resolution = info['resolution']
        # self.ball_radius = info['ball_radius']
        # self.ball_mass = info['ball_mass']
        # self.robot_size = info['robot_size']
        # self.robot_height = info['robot_height']
        # self.axle_length = info['axle_length']
        # self.robot_body_mass = info['robot_body_mass']
        # self.wheel_radius = info['wheel_radius']
        # self.wheel_mass = info['wheel_mass']
        # self.max_linear_velocity = info['max_linear_velocity']
        # self.max_torque = info['max_torque']
        # self.codewords = info['codewords']
        # self.colorChannels = 3
        # self.image = ReceivedImage(self.resolution, self.colorChannels)

    def get_frame(self, f):  # you should override this method
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

        if received_frame.reset_reason == GAME_END:
            # (virtual finish() in random_walk.cpp)
            # save your data
            with open(self.data + '/result.txt', 'w') as output:
                # output.write('yourvariables')
                output.close()
            return False
        return True

    def run(self):
        self.send('get_info')
        info = self.receive()
        self.get_info(json.loads(info))
        self.send('ready')
        while True:
            print('RandomWalk step')
            frame = self.receive()
            if frame:
                if not self.get_frame(json.loads(frame)):  # return False if we need to quit
                    break


class RandomWalkPlayer(Player):
    def get_info(self, info):
        print('get_info data received', info)
        self.number_of_robots = info['number_of_robots']
        self.resolution = info['resolution']
        self.max_linear_velocity = info['max_linear_velocity']
        self.color_channels = 3
        self.image = ReceivedImage(self.resolution, self.color_channels)

    def get_frame(self, frame):
        if 'reset_reason' in frame and frame['reset_reason'] == GAME_END:
            return False
        speeds = []
        for i in range(self.number_of_robots):
            speeds.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
            speeds.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
        self.set_speeds(speeds)
        return True

    def time_callback(self, current_time):
        print("Time callback: %f" % current_time)
        return True



if __name__ == '__main__':

    try:
        unicode
    except NameError:
        # Define 'unicode' for Python 3
        def unicode(s, *_):
            return s

    def to_unicode(s):
        return unicode(s, "utf-8")

    # create a player object and run it
    player = RandomWalkPlayer()
    print('Started random walk')
    player.run()
