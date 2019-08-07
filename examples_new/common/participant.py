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

    # coordinates
    MY_TEAM = 0
    OP_TEAM = 1
    BALL = 2
    X = 0
    Y = 1
    TH = 2
    ACTIVE = 3
    TOUCH = 4


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


class Participant():
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

    def create_frame_object(self, f):
        # initiate empty frame
        frame = Frame()
        if 'time' in f:
            frame.time = f['time']
        if 'score' in f:
            frame.score = f['score']
        if 'reset_reason' in f:
            frame.reset_reason = f['reset_reason']
        if 'game_state' in f:
            frame.game_state = f['game_state']
        if 'ball_ownership' in f:
            frame.ball_ownership = f['ball_ownership']
        if 'half_passed' in f:
            frame.half_passed = f['half_passed']
        if 'subimages' in f:
            frame.subimages = f['subimages']
            # TODO
            # Comment the next lines if you don't need to use the image information
            # for s in frame.subimages:
            #    received_subimages.append(SubImage(s['x'],
            #                                       s['y'],
            #                                       s['w'],
            #                                       s['h'],
            #                                       s['base64'].encode('utf8')))
            # self.image.update_image(received_subimages)
        if 'coordinates' in f:
            frame.coordinates = f['coordinates']
        if 'EOF' in f:
            frame.end_of_frame = f['EOF']

        return frame

    def set_speeds(self, speeds):
        self.send('set_speeds', speeds)

    def send_comment(self, commentary):
        self.send('commentate', commentary)

    def send_report(self, report):
        self.send('report', report)

    def check_frame(self, frame):  # you should override this method
        if frame.reset_reason == Game.GAME_END:
            return False
        return True

    def init(self, info):  # you should override this method
        print("init() method called")

    def update(self, frame):  # you should override this method
        print("update() method called")

    def finish(self):  # you should override this method
        print("finish() method called")

    def run(self):
        self.send('get_info')
        info = self.receive()

        self.init(json.loads(info))

        self.send('ready')
        while True:
            data = self.receive()
            if data:
                # data could contain multiple concatenated frames and last one could not be complete
                try:
                    frames = json.loads("[{}]".format(data.replace('}{', '},{')))
                    finished = False
                    for frame in frames:
                        frameObject = self.create_frame_object(frame)
                        if frame and self.check_frame(frameObject):  # return False if we need to quit
                            self.update(frameObject)
                        else:
                            self.finish(frameObject)
                            finished = True
                            break

                    if finished:
                        break
                except ValueError:
                    sys.stderr.write("Error: participant.py: Invalid JSON object.\n")
