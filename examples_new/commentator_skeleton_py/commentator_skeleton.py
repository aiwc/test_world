#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random

from participant_base import Player, Game, Frame

# shotcuts
X = Player.X
Y = Player.Y
BALL = Player.BALL

class Commentator(Player):
    def init(self, info):
        self.field = info['field']
        self.goal = info['goal']

    def update(self, f):
        # initiate empty frame
        received_frame = Frame()

        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'game_state' in f:
            received_frame.game_state = f['game_state']
        if 'ball_ownership' in f:
            received_frame.ball_ownership = f['ball_ownership']
        if 'half_passed' in f:
            received_frame.half_passed = f['half_passed']
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']
        if 'EOF' in f:
            self.end_of_frame = f['EOF']

        if self.end_of_frame:
            if (received_frame.reset_reason == Game.GAME_START):
                if (not received_frame.half_passed):
                    self.commentate('Game has begun')
                else:
                    self.commentate("Second half has begun")

            elif (received_frame.reset_reason == Game.DEADLOCK):
                self.commentate("Position is reset since no one touched the ball")

            elif (received_frame.reset_reason == Game.GOALKICK):
                self.commentate("A goal kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))

            elif (received_frame.reset_reason == Game.CORNERKICK):
                self.commentate("A corner kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))

            elif (received_frame.reset_reason == Game.PENALTYKICK):
                self.commentate("A penalty kick of Team {}".format("Red" if received_frame.ball_ownership else "Blue"))
            # To get the image at the end of each frame use the variable:
            # self.image.ImageBuffer

            if (received_frame.coordinates[BALL][X] >= (self.field[X] / 2) and abs(received_frame.coordinates[BALL][Y]) <= (self.goal[Y] / 2)):
                self.commentate("Team Red scored!!")
            elif (received_frame.coordinates[BALL][X] <= (-self.field[X] / 2) and abs(received_frame.coordinates[BALL][Y]) <= (self.goal[Y] / 2)):
                self.commentate("Team Blue scored!!")

            if (received_frame.reset_reason == Game.HALFTIME):
                self.commentate("The halftime has met. Current score is: {} : {}".format(received_frame.score[0], received_frame.score[1]))

            if (received_frame.reset_reason == Game.GAME_END):
                if (received_frame.score[0] > received_frame.score[1]):
                    self.commentate("Team Red won the game with score {} : {}".format(received_frame.score[0], received_frame.score[1]))
                elif (received_frame.score[0] < received_frame.score[1]):
                    self.commentate("Team Blue won the game with score {} : {}".format(received_frame.score[1], received_frame.score[0]))
                else:
                    self.commentate("The game ended in a tie with score {} : {}".format(received_frame.score[0], received_frame.score[1]))
            ##############################################################################

        self.end_of_frame = False
        self.previous_frame = received_frame

if __name__ == '__main__':
    player = Commentator()
    player.run()
