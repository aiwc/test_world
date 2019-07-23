#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random

from player import Player


class RandomWalkPlayer(Player):
    def get_info(self, info):
        print('get_info data received', info)
        self.number_of_robots = info['number_of_robots']
        self.max_linear_velocity = info['max_linear_velocity']

    def get_frame(self, frame):
        if not Player.get_frame(self, frame):
            return False
        speeds = []
        for i in range(self.number_of_robots):
            speeds.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
            speeds.append(random.uniform(-self.max_linear_velocity[i], self.max_linear_velocity[i]))
        self.set_speeds(speeds)
        return True


if __name__ == '__main__':
    player = RandomWalkPlayer()
    player.run()
