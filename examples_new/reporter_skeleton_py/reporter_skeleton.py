#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    print(sys.path)
    from participant import Participant
except ImportError as err:
    print('player_random-walk: \'participant\' module cannot be imported:', err)
    raise


class Reporter(Participant):
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
        # TODO
        # self.image = ReceivedImage(self.resolution, self.colorChannels)

        self.paragraphs = []
        print("I am the reporter for this game!")

    def update(self, frame):
        # self.printConsole(frame.time)
        # self.printConsole(frame.score)
        # self.printConsole(frame.reset_reason)
        # self.printConsole(frame.half_passed)
        pass

    def finish(self, frame):
        scoreRed = frame.score[0]
        scoreBlue = frame.score[1]
        if (scoreRed > scoreBlue):
            self.paragraphs.append("Team Red won the game with score {} : {}".format(scoreRed, scoreBlue))
        elif (scoreRed < scoreBlue):
            self.paragraphs.append("Team Blue won the game with score {} : {}".format(scoreBlue, scoreRed))
        else:
            self.paragraphs.append("The game ended in a tie with score {} : {}".format(scoreRed, scoreBlue))

        self.paragraphs.append("It was really a great match!")
        self.send_report(self.paragraphs)


if __name__ == '__main__':
    commentator = Reporter()
    commentator.run()
