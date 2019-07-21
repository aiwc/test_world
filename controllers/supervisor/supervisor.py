#!/usr/bin/env python

import json
import os
import random
import select
import socket
import string
import subprocess
import sys

from controller import Supervisor

import constants


def random_string(length):
    """Generate a random string with the combination of lowercase and uppercase letters."""
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(length))


def get_key(rpc):
    """The key is the first argument of the RPC."""
    first = rpc.find('"') + 1
    return rpc[first:rpc.find('"', first)]


def robot_name(red, id):
    name = constants.DEF_ROBOT_PREFIX
    name += 'R' if red else 'B'
    name += str(id)
    return name


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
        try:
            client.sendall(message.encode())
        except ConnectionAbortedError:
            self.connections.remove(client)

    def ready(self):  # return true when two players are connected
        return len(self.connections) == 3  # contains the server and two players

    def spin(self, game_supervisor):  # handle asynchronous requests from clients
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
                    success = True
                    try:
                        data = s.recv(1024)
                    except socket.error as e:
                        if e.args[0] != 10053:
                            print('Error caught: ', e.args[0])
                        success = False
                    if data and success:
                        game_supervisor.callback(s, data.decode())
                    else:
                        print('Closing')
                        cleanup(s)
            for s in exceptional:
                print('Exceptional')
                cleanup(s)


class GameSupervisor (Supervisor):
    timeStep = 10

    def __init__(self):
        Supervisor.__init__(self)
        self.receiver = self.getReceiver(constants.NAME_RECV)
        self.cameraANode = self.getFromDef(constants.DEF_CAMA)
        self.cameraBNode = self.getFromDef(constants.DEF_CAMB)
        self.viewpointNode = self.getFromDef(constants.DEF_AUDVIEW)
        self.cameraA = self.getCamera(constants.NAME_CAMA)
        self.cameraA = self.getCamera(constants.NAME_CAMB)

        # DEF_GRASS is not visible to cam a and cam b, optional
        grass = self.getFromDef(constants.DEF_GRASS)
        grass.setVisibility(self.cameraANode, False)
        grass.setVisibility(self.cameraBNode, False)
        # BALLSHAPE is visible only to viewpoint, ORANGESHAPE is to cam_a and cam_b, mandatory
        ball = self.getFromDef(constants.DEF_BALLSHAPE)
        orange = self.getFromDef(constants.DEF_ORANGESHAPE)
        ball.setVisibility(self.cameraANode, False)
        ball.setVisibility(self.cameraBNode, False)
        if orange:
            orange.setVisibility(self.viewpointNode, False)
        # Stadium is visible only to viewpoint, optional
        stadium = self.getFromDef(constants.DEF_STADIUM)
        if stadium:
            stadium.setVisibility(self.cameraANode, False)
            stadium.setVisibility(self.cameraBNode, False)
        # Robot's gray cover is visible only to robots
        for team in range(0, 2):
            for id in range(0, constants.NUMBER_OF_ROBOTS):
                print(robot_name(True if team == 0 else False, id))
                robot = self.getFromDef(robot_name(team == 0, id))
                cover = robot.getField('cover')
                cover0 = cover.getMFNode(0)
                cover0.setVisibility(self.viewpointNode, False)
        # Wall is visible only to robots
        wall = self.getFromDef(constants.DEF_WALL)
        if wall:
            wall.setVisibility(self.viewpointNode, False)
        # VisualWall is visible only to viewpoint, optional
        visual_wall = self.getFromDef(constants.DEF_VISWALL)
        if visual_wall:
            visual_wall.setVisibility(self.cameraANode, False)
            visual_wall.setVisibility(self.cameraBNode, False)
        # patches'
        for team in range(0, 2):
            for id in range(0, constants.NUMBER_OF_ROBOTS):
                robot = self.getFromDef(robot_name(team == 0, id))
                patches = robot.getField('patches')
                # number patch for decoration exists
                if patches.getCount() == 3:
                    number = patches.getMFNode(0)
                    id_red = patches.getMFNode(1)
                    id_blue = patches.getMFNode(2)
                    number.setVisibility(self.cameraANode, False)
                    number.setVisibility(self.cameraBNode, False)
                    id_red.setVisibility(self.viewpointNode, False)
                    id_red.setVisibility(self.cameraBNode, False)
                    id_blue.setVisibility(self.viewpointNode, False)
                    id_blue.setVisibility(self.cameraANode, False)
                else:  # no decorations
                    id_red = patches.getMFNode(0)
                    id_blue = patches.getMFNode(1)
                    id_red.setVisibility(self.viewpointNode, True)  # useless ?
                    id_red.setVisibility(self.cameraBNode, False)
                    id_blue.setVisibility(self.viewpointNode, False)
                    id_blue.setVisibility(self.cameraANode, False)

    def get_team_color(self, rpc):
        if self.team_info['T_RED']['key'] == get_key(rpc):
            return 'T_RED'
        else:
            return 'T_BLUE'

    def callback(self, client, message):
        if not message.startswith('aiwc.'):
            print('Error, AIWC RPC messages should start with "aiwc.".')
            return
        message = message[5:]
        color = self.get_team_color(message)
        self.team_client[color] = client
        if message.startswith('get_info('):
            print('Server receive aiwc.get_info from ' + color)
            client.sendall(json.dumps(self.team_info[color]).encode())
        elif message.startswith('ready('):
            self.ready[color] = True
            print('Server receive aiwc.ready from ' + color)
            client.sendall(b'OK')
        elif message.startswith('set_speeds('):
            start = message.find('",') + 2
            end = message.find(')', start)
            speed = message[start:end]
            speed = [float(i) for i in speed.split(',')]
            print('speeds =', speed)
            def_robot_prefix = constants.DEF_ROBOT_PREFIX + color[2]
            for i in range(0, 5):
                robot = self.getFromDef(def_robot_prefix + str(i))
                robot.getField('customData').setSFString("%f %f" % (speed[i * 2], speed[i * 2 + 1]))
        else:
            print('Server received unknown message', message)

    def send(self, color, message):
        self.team_client[color].sendall(message.encode())

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
        player_team_infos = []
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
        team_name = {}
        self.team_info = {}
        self.team_client = {}
        self.ready = {}
        self.ready['T_RED'] = False
        self.ready['T_BLUE'] = False
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
            player_team_infos.append([name, rating, path_prefix + exe, path_prefix, 'ROLE_PLAYER', team == 'T_RED'])
            if team == 'T_RED':
                print('Team A:\n')
            else:
                print('Team B:\n')
            print('  team name - ' + name + '\n')
            team_name[team] = name
            print(' executable - ' + exe + '\n')
            print('  data path - ' + data + '\n\n')

            # create information for aiwc.get_info() in advance
            info = {}
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
            info['team_info'] = [[['name', name], ['rating', rating]], [['name', name_op], ['rating', rating_op]]]
            info['key'] = random_string(constants.KEY_LENGTH)
            self.team_info[team] = info

        # gets commentator information from 'config.json' (commentator is optional)
        if config['commentator']:
            if config['commentator']['name']:
                name = config['commentator']['name']
            if config['commentator']['executable']:
                exe = config['commentator']['executable']
            if config['commentator']['datapath']:
                data = config['commentator']['datapath']
            if exe:  # commentator is treated as red team with rating 0
                player_team_infos.append([name, 0, path_prefix + exe, path_prefix + data, 'ROLE_COMMENTATOR', True])
                print('Commentator:\n')
                print('  team name - ' + name + '\n')
                print(' executable - ' + exe + '\n')
                print('  data path - ' + data + '\n\n')
            else:
                print('Commentator "executable" is missing: skipping commentator\n')
        else:
            print('"commentator" section of \'config.json\' seems to be missing: skipping commentator\n')

        #  gets reporter information from 'config.json' (reporter is optional)
        if config['reporter']:
            if config['reporter']['name']:
                name = config['reporter']['name']
            if config['reporter']['executable']:
                exe = config['reporter']['executable']
            if config['reporter']['datapath']:
                data = config['reporter']['datapath']
            if exe:  # reporter is treated as red team with rating 0
                player_team_infos.append([name, 0, path_prefix + exe, path_prefix + data, 'ROLE_REPORTER', True])
                print('Reporter:\n')
                print('  team name - ' + name + '\n')
                print(' executable - ' + exe + '\n')
                print('  data path - ' + data + '\n\n')
            else:
                print('Reporter "executable" is missing: skipping reporter\n')
        else:
            print('"reporter" section of \'config.json\' seems to be missing: skipping reporter\n')

        self.tcp_server = TcpServer(constants.SERVER_IP, constants.SERVER_PORT)

        # start participants
        for player_team_info in player_team_infos:
            exe = player_team_info[2]
            color = 'T_RED' if player_team_info[5] else 'T_BLUE'
            if not os.path.exists(exe):
                print('Participant controller not found: ' + exe)
            else:
                command_line = []
                if exe.endswith('.py'):
                    command_line.append('python')
                command_line.append(exe)
                command_line.append(constants.SERVER_IP)
                command_line.append(str(constants.SERVER_PORT))
                command_line.append(self.team_info[color]['key'])
                command_line.append(player_team_info[3])
                print(command_line)
                subprocess.Popen(command_line)
        self.started = False
        print('Waiting for player to be ready...')
        ball = self.getFromDef(constants.DEF_BALL)
        while True:
            sys.stdout.flush()
            self.tcp_server.spin(self)
            if not self.started:
                if self.ready['T_RED'] and self.ready['T_BLUE']:
                    print('Starting match.')
                    self.started = True
            else:
                for team in ['T_RED', 'T_BLUE']:
                    # here we should sends more information than just ball position
                    ball_position = ball.getPosition()
                    self.send(team, json.dumps(ball_position))
            if self.step(self.timeStep) == -1:
                break


controller = GameSupervisor()
controller.run()
