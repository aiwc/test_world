import json
import socket
import sys

import base64
import numpy as np


class Frame():
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.game_state = None
        self.subimages = None
        self.coordinates = None
        self.half_passed = None


class Game():
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


class ReceivedImage(object):
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
                    y = j + self.received_parts[i].y
                    x = k + self.received_parts[i].x
                    self.ImageBuffer[y, x, 0] = reshaped_msg[j, k, 0]  # blue channel
                    self.ImageBuffer[y, x, 1] = reshaped_msg[j, k, 1]  # green channel
                    self.ImageBuffer[y, x, 2] = reshaped_msg[j, k, 2]  # red channel


class SubImage():
    def __init__(self, x, y, width, height, b64):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.b64 = b64


class Commentator():
    def __init__(self):
        self.host = sys.argv[1]
        self.port = int(sys.argv[2])
        self.key = sys.argv[3]
        self.datapath = sys.argv[4]
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
        data = self.socket.recv(4096)
        return data.decode()

    def set_comment(self, commentary):
        self.send('commentate', commentary)

    def check_frame(self, frame):  # you should override this method
        if 'reset_reason' in frame and frame['reset_reason'] == Game.GAME_END:
            return False
        return True

    def init(self, info):  # you should override this method
        print("init() method called")

    def update(self, frame):  # you should override this method
        print("update() method called")

    def finish(self, frame):  # you should override this method
        print("finish() method called")

    def run(self):
        self.send('get_info')
        info = self.receive()

        self.init(json.loads(info))

        self.send('ready')

        while True:
            frame = self.receive()
            if frame and self.check_frame(json.loads(frame)):  # return False if we need to quit
                self.update(json.loads(frame))
            else:
                self.finish(frame)
                break
