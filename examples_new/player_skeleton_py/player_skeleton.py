#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random

from player import Player


class RandomWalkPlayer(Player):
    def init(self, info):
        self.number_of_robots = info['number_of_robots']
        self.max_linear_velocity = info['max_linear_velocity']

    def update(self, frame):
        speeds = []
        for i in range(self.number_of_robots):
            speeds.append(self.max_linear_velocity[i])
            speeds.append(self.max_linear_velocity[i])
        self.set_speeds(speeds)


if __name__ == '__main__':
    player = RandomWalkPlayer()
    player.run()
