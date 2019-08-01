import json
import socket
import sys


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


class Player():
    # coordinates
    MY_TEAM = 0
    OP_TEAM = 1
    BALL = 2
    X = 0
    Y = 1
    TH = 2
    ACTIVE = 3
    TOUCH = 4

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
        data = self.socket.recv(4096)
        return data.decode()

    def set_speeds(self, speeds):
        self.send('set_speeds', speeds)

    def check_frame(self, frame):  # you should override this method
        if 'reset_reason' in frame and frame['reset_reason'] == Game.GAME_END:
            return False
        return True

    def init(self, info):  # you should override this method
        print("init() method called")

    def update(self, frame): # you should override this method
        print("update() method called")

    def finish(self): # you should override this method
        print("finish() method called")

    def run(self):
        self.send('get_info')
        info = self.receive()

        self.init(json.loads(info))

        self.send('ready')

        while True:
            frame = self.receive()
            if frame and self.check_frame(json.loads(frame)): # return False if we need to quit
                self.update(json.loads(frame))
            else:
                self.finish()
                break
