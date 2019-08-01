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

    def get_info(self, info):  # you should override this method
        print("get_info() method called", info)
        # Here you have the information of the game
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
        # self.color_channels = 3
        # self.image = ReceivedImage(self.resolution, self.colorChannels)

    def get_frame(self, frame):  # you should override this method
        if 'reset_reason' in frame and frame['reset_reason'] == Game.GAME_END:
            return False
        return True

    def run(self):
        self.send('get_info')
        info = self.receive()
        self.get_info(json.loads(info))
        self.send('ready')
        while True:
            frame = self.receive()
            if not frame or not self.get_frame(json.loads(frame)):  # return False if we need to quit
                break
