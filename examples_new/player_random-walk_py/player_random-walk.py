#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    print(sys.path)
    from participant import Participant
except ImportError as err:
    print('player_random-walk: \'participant\' module cannot be imported:', err)
    raise


class RandomWalkPlayer(Participant):
    def init(self, info):
        self.number_of_robots = info['number_of_robots']
        self.max_linear_velocity = info['max_linear_velocity']

    def update(self, frame):
        speeds = []
        for i in range(self.number_of_robots):
            speeds.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
            speeds.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
        self.set_speeds(speeds)


if __name__ == '__main__':
    player = RandomWalkPlayer()
    player.run()
