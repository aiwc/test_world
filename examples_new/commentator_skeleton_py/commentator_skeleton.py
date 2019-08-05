#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

from commentator import Commentator, Frame, Game, SubImage, ReceivedImage

# shortcuts
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2
ACTIVE = 3
TOUCH = 4


class BasicCommentator(Commentator):
    def init(self, info):
        # Here you have the information of the game (virtual init() in random_walk.cpp)
        # List: game_time, number_of_robots
        #       field, goal, penalty_area, goal_area, resolution Dimension: [x, y]
        #       ball_radius, ball_mass,
        #       robot_size, robot_height, axle_length, robot_body_mass, ID: [0, 1, 2, 3, 4]
        #       wheel_radius, wheel_mass, ID: [0, 1, 2, 3, 4]
        #       max_linear_velocity, max_torque, codewords, ID: [0, 1, 2, 3, 4]
        # self.game_time = info['game_time']
        # self.number_of_robots = info['number_of_robots']

        self.field = info['field']
        self.goal = info['goal']
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

        # self.max_linear_velocity = info['max_linear_velocity']
        # self.max_torque = info['max_torque']
        # self.codewords = info['codewords']

        self.colorChannels = 3
        self.end_of_frame = False
        self.image = ReceivedImage(self.resolution, self.colorChannels)

        print("I am the commentator for this game!")

    def update(self, frame):
        # initiate empty frame
        received_frame = Frame()
        received_subimages = []
        if 'time' in frame:
            received_frame.time = frame['time']
        if 'score' in frame:
            received_frame.score = frame['score']
        if 'reset_reason' in frame:
            received_frame.reset_reason = frame['reset_reason']
        if 'half_passed' in frame:
            received_frame.half_passed = frame['half_passed']
        if 'ball_ownership' in frame:
            received_frame.ball_ownership = frame['ball_ownership']
        if 'subimages' in frame:
            received_frame.subimages = frame['subimages']
            # Comment the next lines if you don't need to use the image information
            for s in received_frame.subimages:
                received_subimages.append(SubImage(s['x'],
                                                   s['y'],
                                                   s['w'],
                                                   s['h'],
                                                   s['base64'].encode('utf8')))
            self.image.update_image(received_subimages)
        if 'coordinates' in frame:
            received_frame.coordinates = frame['coordinates']

        # self.printConsole(received_frame.time)
        # self.printConsole(received_frame.score)
        # self.printConsole(received_frame.reset_reason)
        # self.printConsole(received_frame.half_passed)
        # self.printConsole(self.end_of_frame)

        if (received_frame.reset_reason == Game.GAME_START):
            if not received_frame.half_passed:
                self.set_comment("Game has begun")
            else:
                self.set_comment("Second half has begun")

        elif received_frame.reset_reason == Game.DEADLOCK:
            self.set_comment("Position is reset since no one touched the ball")

        elif received_frame.reset_reason == Game.GOALKICK:
            self.set_comment("A goal kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))

        elif received_frame.reset_reason == Game.CORNERKICK:
            self.set_comment("A corner kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))

        elif received_frame.reset_reason == Game.PENALTYKICK:
            self.set_comment("A penalty kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))
        # To get the image at the end of each frame use the variable:
        # self.image.ImageBuffer

        if (received_frame.coordinates[BALL][X] >= (self.field[X] / 2) and
                abs(received_frame.coordinates[BALL][Y]) <= (self.goal[Y] / 2)):
            self.set_comment("Team Red scored!!")
        elif (received_frame.coordinates[BALL][X] <= (-self.field[X] / 2) and
                abs(received_frame.coordinates[BALL][Y]) <= (self.goal[Y] / 2)):
            self.set_comment("Team Blue scored!!")

        if received_frame.reset_reason == Game.HALFTIME:
            self.set_comment("The halftime has met. Current score is: {} : {}".format(
                received_frame.score[0], received_frame.score[1]))

    def finish(self, frame):
        if (frame.score[0] > frame.score[1]):
            self.set_comment("Team Red won the game with score {} : {}".format(frame.score[0], frame.score[1]))
        elif (frame.score[0] < frame.score[1]):
            self.set_comment("Team Blue won the game with score {} : {}".format(frame.score[1], frame.score[0]))
        else:
            self.set_comment("The game ended in a tie with score {} : {}".format(frame.score[0], frame.score[1]))

        # save your data
        with open(self.datapath + '/result.txt', 'w') as output:
            # output.write('yourvariables')
            output.close()


if __name__ == '__main__':
    commentator = BasicCommentator()
    commentator.run()
