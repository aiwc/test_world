#!/usr/bin/env python

import json
import select
import socket

import constants

from controller import Supervisor


class TcpServer:
    def __init__(self, host, port):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setblocking(False)
        self.server.bind((host, port))
        self.server.listen(5)
        self.connections = [self.server]

    def send_to_all(self, message):  # send message to all clients
        for client in self.connections:
            if client == self.server:
                continue
            self.send(client, message)

    def send(self, client, message):  # send message to a single client
        client.sendall(message.encode('utf-8'))

    def spin(self):  # handle asynchronous requests from clients
        def cleanup(s):
            print('Cleanup')
            if s in self.connections:
                self.connections.remove(s)
            s.close()
        while True:
            readable, writable, exceptional = select.select(self.connections, [], self.connections, 0)
            if not readable and not writable and not exceptional:
                return
            for s in readable:
                if s is self.server:
                    connection, client_address = s.accept()
                    connection.setblocking(False)
                    self.connections.append(connection)
                    print('Accepted ', client_address)
                else:
                    try:
                        data = s.recv(1024)
                    except socket.error as e:
                        print('Error caught: ', e.args[0])
                    if data:
                        print('Received data: ', data)
                        self.send(s, data.decode('utf-8').lower())
                    else:
                        print('Closing')
                        cleanup(s)
            for s in exceptional:
                print('Exceptional')
                cleanup(s)


class GameSupervisor (Supervisor):
    timeStep = 10

    def run(self):
        config_file = open('../../config.json')
        config = json.loads(config_file.read())
        game_time_ms = constants.DEFAULT_GAME_TIME_MS / constants.PERIOD_MS * constants.PERIOD_MS
        deadlock_flag = True
        if config['rule']:
            if config['rule']['game_time']:
                game_time_ms = config['rule']['game_time'] * 1000 / constants.PERIOD_MS * constants.PERIOD_MS
            if config['rule']['deadlock']:
                deadlock_flag = config['rule']['deadlock']
        else:
            print('"rule" section of \'config.json\' seems to be missing: using default options\n')
        print('Rules:\n')
        print('     game duration - ' + str(game_time_ms / 1000) + ' seconds\n')
        print('          deadlock - ' + str(deadlock_flag) + '\n')

        # gets other options from 'config.json' (if no option is specified, default option is given)
        # automatic recording of the game (default: false)
        # automatic repetition of the game (default: false)
        repeat = False
        record = False
        record_path = ''
        if config['tool']:
            if config['tool']['repeat']:
                repeat = config['tool']['repeat']
            if repeat:  # if repeat is enabled, record is forced to be disabled
                print('Game repetition is enabled that the game recording will be disabled.\n')
            elif config['tool']['record']:
                record = config['tool']['record']
            if config['tool']['record_path']:
                record_path = config['tool']['record_path']
            path_prefix = '../../'

        # gets the teams' information from 'config.json'
        for team in ['T_RED', 'T_BLUE']:
            if team == 'T_RED':
                tc = 'team_a'
                tc_op = 'team_b'
            else:
                tc = 'team_b'
                tc_op = 'team_a'
        # my team
        name = ''
        rating = 0  # rating is disabled
        exe = ''
        data = ''
        if config[tc]:
            if config[tc]['name']:
                name = config[tc]['name']
            if config[tc]['executable']:
                exe = config[tc]['executable']
            if config[tc]['datapath']:
                data = config[tc]['datapath']
        # opponent
        name_op = ''
        rating_op = 0  # rating is disabled
        if config[tc_op]:
            if config[tc_op]['name']:
                name_op = config[tc_op]['name']
        player_team_infos = {name, rating, path_prefix + exe, path_prefix, 'ROLE_PLAYER', team == 'T_RED'}
        if team == 'T_RED':
            print('Team A:\n')
        else:
            print('Team B:\n')
        print('  team name - ' + name + '\n')
        team_name[team] = name
        print(' executable - ' + exe + '\n')
        print('  data path - ' + data + '\n\n')

        # create information for aiwc.get_info() in advance
        info['field'] = [constants.FIELD_LENGTH, constants.FIELD_WIDTH]
        info['goal'] = [constants.GOAL_DEPTH, constants.GOAL_WIDTH]
        info['penalty_area'] = [constants.PENALTY_AREA_DEPTH, constants.PENALTY_AREA_WIDTH]
        info['goal_area'] = [constants.GOAL_AREA_DEPTH, constants.GOAL_AREA_WIDTH]
        info['ball_radius'] = constants.BALL_RADIUS
        info['ball_mass'] = constants.BALL_MASS
        info['robot_size'] = constants.ROBOT_SIZE
        info['robot_height'] = constants.ROBOT_HEIGHT
        info['axle_length'] = constants.AXLE_LENGTH
        info['robot_body_mass'] = constants.ROBOT_BODY_MASS
        info['wheel_radius'] = constants.WHEEL_RADIUS
        info['wheel_mass'] = constants.WHEEL_MASS
        info['max_linear_velocity'] = constants.MAX_LINEAR_VELOCITY
        info['max_torque'] = constants.MAX_TORQUE
        info['resolution'] = [constants.RESOLUTION_X, constants.RESOLUTION_Y]
        info['number_of_robots'] = constants.NUMBER_OF_ROBOTS
        info['codewords'] = constants.CODEWORDS
        info['game_time'] = game_time_ms / 1000
        info['team_info'] = [{{'name', name}, {'rating', rating}}, {{'name', name_oc}, {'rating', rating_oc}}]

        # gets commentator information from 'config.json' (commentator is optional)
        if config['commentator']:
            if config['commentator']['name']:
                name = config['commentator']['name']
            if config['commentator']['executable']:
                exe = config['commentator']['executable']
            if config['commentator']['datapath']:
                data = config['commentator']['datapath']
            if exe:  # commentator is treated as red team with rating 0
                player_team_infos = {name, 0, path_prefix + exe, path_prefix + data, 'ROLE_COMMENTATOR', True}
                print('Commentator:\n')
                print('  team name - ' + name + '\n')
                print(' executable - ' + exe + '\n')
                print('  data path - ' + data + '\n\n')
            else:
                print('Commentator "executable" is missing: skipping commentator\n')
        else:
            print('"commentator" section of \'config.json\' seems to be missing: skipping commentator\n')

        tcp_server = TcpServer(constants.SERVER_IP, constants.SERVER_PORT)
        while True:
            tcp_server.spin()
            if self.step(self.timeStep) == -1:
                break


controller = GameSupervisor()
controller.run()
