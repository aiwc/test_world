#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    print(sys.path)
    from participant import Participant, Game, Frame
except ImportError as err:
    print('commentator_skeleton: \'participant\' module cannot be imported:', err)
    raise


class Commentator(Participant):
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
        # TODO self.image = ReceivedImage(self.resolution, self.colorChannels)

        print("I am the commentator for this game!")

    def commentate(self, commentary):
        self.send_comment([commentary])

    def update(self, received_frame):
        # print(received_frame.time)
        # print(received_frame.score)
        # print(received_frame.reset_reason)
        # print(received_frame.half_passed)
        # print(self.end_of_frame)

        if (received_frame.reset_reason == Game.GAME_START):
            if not received_frame.half_passed:
                self.commentate("Game has begun")
            else:
                self.commentate("Second half has begun")

        elif received_frame.reset_reason == Game.DEADLOCK:
            self.commentate("Position is reset since no one touched the ball")

        elif received_frame.reset_reason == Game.GOALKICK:
            self.commentate("A goal kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))

        elif received_frame.reset_reason == Game.CORNERKICK:
            self.commentate("A corner kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))

        elif received_frame.reset_reason == Game.PENALTYKICK:
            self.commentate("A penalty kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))
        # To get the image at the end of each frame use the variable:
        # self.image.ImageBuffer

        if (received_frame.coordinates[Frame.BALL][Frame.X] >= (self.field[Frame.X] / 2) and
                abs(received_frame.coordinates[Frame.BALL][Frame.Y]) <= (self.goal[Frame.Y] / 2)):
            self.commentate("Team Red scored!!")
        elif (received_frame.coordinates[Frame.BALL][Frame.X] <= (-self.field[Frame.X] / 2) and
                abs(received_frame.coordinates[Frame.BALL][Frame.Y]) <= (self.goal[Frame.Y] / 2)):
            self.commentate("Team Blue scored!!")

        if received_frame.reset_reason == Game.HALFTIME:
            self.commentate("The halftime has met. Current score is: {} : {}".format(
                received_frame.score[0], received_frame.score[1]))

    def finish(self, frame):
        scoreRed = frame.score[0]
        scoreBlue = frame.score[1]
        if (scoreRed > scoreBlue):
            self.commentate("Team Red won the game with score {} : {}".format(scoreRed, scoreBlue))
        elif (scoreRed < scoreBlue):
            self.commentate("Team Blue won the game with score {} : {}".format(scoreBlue, scoreRed))
        else:
            self.commentate("The game ended in a tie with score {} : {}".format(scoreRed, scoreBlue))

        # save your data
        with open(self.datapath + '/result.txt', 'w') as output:
            # output.write('yourvariables')
            output.close()


if __name__ == '__main__':
    commentator = Commentator()
    commentator.run()
