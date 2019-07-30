#!/usr/bin/env python

import json
import math
import os
import random
import select
import socket
import string
import subprocess
import sys

from controller import Supervisor

from player_py.player import Game

import constants


def random_string(length):
    """Generate a random string with the combination of lowercase and uppercase letters."""
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(length))


def get_key(rpc):
    """The key is the first argument of the RPC."""
    first = rpc.find('"') + 1
    return rpc[first:rpc.find('"', first)]


def robot_name(color, id):
    name = constants.DEF_ROBOT_PREFIX
    name += 'R' if color == 0 else 'B'
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
        if client.fileno() == -1:  # was closed
            return
        try:
            client.sendall(message.encode())
        except socket.ConnectionAbortedError:
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
                    data = None
                    try:
                        data = s.recv(4096)
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
        self.receiver.enable(constants.RECV_PERIOD_MS)

        self.cameraA = self.getCamera(constants.NAME_CAMA)
        self.cameraA.enable(constants.CAM_PERIOD_MS)
        self.cameraB = self.getCamera(constants.NAME_CAMB)
        self.cameraB.enable(constants.CAM_PERIOD_MS)

        self.cameraANode = self.getFromDef(constants.DEF_CAMA)
        self.cameraBNode = self.getFromDef(constants.DEF_CAMB)
        self.viewpointNode = self.getFromDef(constants.DEF_AUDVIEW)
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
        for team in range(2):
            for id in range(constants.NUMBER_OF_ROBOTS):
                robot = self.getFromDef(robot_name(team, id))
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
        for team in range(2):
            for id in range(constants.NUMBER_OF_ROBOTS):
                robot = self.getFromDef(robot_name(team, id))
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
        if self.team_info[0]['key'] == get_key(rpc):
            return 0
        else:
            return 1

    def set_speeds(self, team, speeds):
        letter = 'R' if team == 0 else 'B'
        def_robot_prefix = constants.DEF_ROBOT_PREFIX + letter
        for id in range(5):
            robot = self.getFromDef(def_robot_prefix + str(id))
            if self.robot[team][id]['active']:
                robot.getField('customData').setSFString("%f %f" % (speeds[id * 2], speeds[id * 2 + 1]))

    def callback(self, client, message):
        if not message.startswith('aiwc.'):
            print('Error, AIWC RPC messages should start with "aiwc.".')
            return
        message = message[5:]
        color = self.get_team_color(message)
        self.team_client[color] = client
        if message.startswith('get_info('):
            print('Server receive aiwc.get_info from team ' + str(color))
            self.tcp_server.send(client, json.dumps(self.team_info[color]))
        elif message.startswith('ready('):
            self.ready[color] = True
            print('Server receive aiwc.ready from team ' + str(color))
        elif message.startswith('set_speeds('):
            start = message.find('",') + 2
            end = message.find(')', start)
            speeds = message[start:end]
            speeds = [float(i) for i in speeds.split(',')]
            self.set_speeds(color, speeds)
        else:
            print('Server received unknown message', message)

    def reset_ball(self, x, z):
        f = -1.0 if self.half_passed else 1.0
        self.ball.getField('translation').setSFVec3f([f * x, 1.5 * constants.BALL_RADIUS, -f * z])
        self.ball.getField('rotation').setSFRotation([0, 1, 0, 0])
        self.ball.resetPhysics()

    def reset_robot(self, team, id, x, y, z, th):
        robot = self.getFromDef(robot_name(team, id))
        f = -1 if self.half_passed else 1
        translation = [f * x, y, f * -z]
        rotation = [0, 1, 0, th + constants.PI if self.half_passed else 0]

        al = robot.getField('axleLength').getSFFloat()
        h = robot.getField('height').getSFFloat()
        wr = robot.getField('wheelRadius').getSFFloat()

        lwTranslation = [-al / 2, (-h + 2 * wr) / 2, 0]
        rwTranslation = [al / 2, (-h + 2 * wr) / 2, 0]
        wheelRotation = [1, 0, 0, constants.PI / 2]

        robot.getField('translation').setSFVec3f(translation)
        robot.getField('rotation').setSFRotation(rotation)
        robot.getField('lwTranslation').setSFVec3f(lwTranslation)
        robot.getField('lwRotation').setSFRotation(wheelRotation)
        robot.getField('rwTranslation').setSFVec3f(rwTranslation)
        robot.getField('rwRotation').setSFRotation(wheelRotation)
        robot.resetPhysics()
        self.robot[team][id]['x'] = translation[0]
        self.robot[team][id]['y'] = translation[2]
        self.robot[team][id]['th'] = rotation[3]
        self.robot[team][id]['active'] = True
        self.robot[team][id]['touch'] = False
        self.robot[team][id]['fall_time'] = 0
        self.robot[team][id]['sent_out_time'] = 0
        self.deadlock_time = self.getTime()
        self.set_speeds(team, [0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def reset(self, red_formation, blue_formation):
        # reset the ball
        if red_formation == constants.ROBOT_DEFAULT or red_formation == constants.ROBOT_KICKOFF:
            self.reset_ball(constants.BALL_POSTURE[constants.BALL_DEFAULT][0],
                            constants.BALL_POSTURE[constants.BALL_DEFAULT][1])
        elif red_formation == constants.GOALKICK_A:
            self.reset_ball(constants.BALL_POSTURE[constants.BALL_GOALKICK][0],
                            constants.BALL_POSTURE[constants.BALL_GOALKICK][1])
        elif red_formation == constants.GOALKICK_D:
            self.reset_ball(-constants.BALL_POSTURE[constants.BALL_GOALKICK][0],
                            constants.BALL_POSTURE[constants.BALL_GOALKICK][1])
        elif red_formation == constants.CAD_AD or red_formation == constants.CAD_DA:
            self.reset_ball(constants.BALL_POSTURE[constants.BALL_CORNERKICK][0],
                            constants.BALL_POSTURE[constants.BALL_CORNERKICK][1])
        elif red_formation == constants.CBC_AD or red_formation == constants.CBC_DA:
            self.reset_ball(constants.BALL_POSTURE[constants.BALL_CORNERKICK][0],
                            -constants.BALL_POSTURE[constants.BALL_CORNERKICK][1])
        elif red_formation == constants.CAD_AA or red_formation == constants.CAD_DD:
            self.reset_ball(-constants.BALL_POSTURE[constants.BALL_CORNERKICK][0],
                            -constants.BALL_POSTURE[constants.BALL_CORNERKICK][1])
        elif red_formation == constants.CBC_AA or red_formation == constants.CBC_DD:
            self.reset_ball(-constants.BALL_POSTURE[constants.BALL_CORNERKICK][0],
                            constants.BALL_POSTURE[constants.BALL_CORNERKICK][1])
        elif red_formation == constants.PENALTYKICK_A:
            self.reset_ball(constants.BALL_POSTURE[constants.BALL_PENALTYKICK][0],
                            constants.BALL_POSTURE[constants.BALL_PENALTYKICK][1])
        elif red_formation == constants.PENALTYKICK_D:
            self.reset_ball(-constants.BALL_POSTURE[constants.BALL_PENALTYKICK][0],
                            constants.BALL_POSTURE[constants.BALL_PENALTYKICK][1])

        # reset the robots
        for team in range(2):
            if team == 0:
                s = 1
                a = 0
                formation = red_formation
            else:
                s = -1
                a = constants.PI
                formation = blue_formation
            for id in range(constants.NUMBER_OF_ROBOTS):
                self.reset_robot(team, id,
                                 constants.ROBOT_FORMATION[formation][id][0] * s,
                                 constants.ROBOT_HEIGHT[id] / 2,
                                 constants.ROBOT_FORMATION[formation][id][1] * s,
                                 constants.ROBOT_FORMATION[formation][id][2] + a - constants.PI / 2)

    def update_positions(self):
        for t in range(2):
            for id in range(constants.NUMBER_OF_ROBOTS):
                node = self.robot[t][id]['node']
                position = node.getPosition()
                self.robot[t][id]['x'] = position[0]
                self.robot[t][id]['y'] = position[2]
                orientation = node.getOrientation()
                self.robot[t][id]['th'] = orientation[3]
        self.ball_position = self.ball.getPosition()

    def generate_frame(self, team):
        opponent = 1 if team == 0 else 0
        frame = {}
        frame['time'] = self.getTime()
        frame['score'] = [self.score[team], self.score[opponent]]
        frame['reset_reason'] = self.reset_reason
        frame['game_state'] = self.game_state
        frame['ball_ownership'] = True if self.ball_ownership == team else False
        frame['half_passed'] = self.half_passed
        frame['subimages'] = []
        frame['coordinates'] = [None] * 3
        for t in range(2):
            frame['coordinates'][t] = [None] * constants.NUMBER_OF_ROBOTS
            c = team if t == 0 else opponent
            for id in range(constants.NUMBER_OF_ROBOTS):
                frame['coordinates'][t][id] = [None] * 5
                frame['coordinates'][t][id][0] = self.robot[c][id]['x']
                frame['coordinates'][t][id][1] = self.robot[c][id]['y']
                frame['coordinates'][t][id][2] = self.robot[c][id]['th']
                frame['coordinates'][t][id][3] = self.robot[c][id]['active']
                frame['coordinates'][t][id][4] = self.robot[c][id]['touch']
        frame['coordinates'][2] = [None] * 2
        frame['coordinates'][2][0] = self.ball_position[0]
        frame['coordinates'][2][1] = self.ball_position[2]
        frame['EOF'] = True
        return frame

    def lock_all_robots(self, locked):
        for t in range(2):
            for id in range(constants.NUMBER_OF_ROBOTS):
                self.robot[t][id]['active'] = not locked

    def stop_robots(self):
        for t in range(2):
            self.set_speeds(t, [0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def update_label(self):
        if not self.half_passed:
            self.setLabel(1, '1st Half', 0.45, 0.9, 0.10, 0x00000000, 0, 'Arial')
            self.setLabel(0, 'score %d:%d, time %.2f' % (self.score[0], self.score[1], self.time / 1000),
                          0.4, 0.95, 0.1, 0x00000000, 0, 'Arial')
        else:
            self.setLabel(1, '2nd Half', 0.45, 0.9, 0.10, 0x00000000, 0, 'Arial')
            self.setLabel(0, 'score %d:%d, time %.2f' % (self.score[1], self.score[0], (self.game_time + self.time) / 1000),
                          0.4, 0.95, 0.10, 0x00000000, 0, 'Arial')

    def mark_half_passed(self):
        self.cameraANode.getField('rotation').setSFRotation([0, 0, 1, constants.PI])
        self.cameraBNode.getField('rotation').setSFRotation([0, 0, 1, 0])

    def episode_restart(self):
        self.cameraANode.getField('rotation').setSFRotation([0, 0, 1, 0])
        self.cameraBNode.getField('rotation').setSFRotation([0, 0, 1, constants.PI])

    def get_robot_touch_ball(self):
        rc = [[False] * constants.NUMBER_OF_ROBOTS, [False] * constants.NUMBER_OF_ROBOTS]
        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getData()
            for team in range(2):
                for id in range(constants.NUMBER_OF_ROBOTS):
                    if message[2 * id + team] == '1':
                        rc[team][id] = True
            self.receiver.nextPacket()
        return rc

    def get_robot_posture(self, team, id):
        position = self.robot[team][id]['node'].getPosition()
        orientation = self.robot[team][id]['node'].getOrientation()
        f = -1 if self.half_passed else 1
        x = position[0]
        y = -position[2]
        th = (constants.PI if self.half_passed else 0) + math.atan2(orientation[2], orientation[8]) + constants.PI / 2
        # Squeeze the orientation range to [-PI, PI]
        while th > constants.PI:
            th -= 2 * constants.PI
        while th < -constants.PI:
            th += 2 * constants.PI
        stand = orientation[4] > 0.8
        return [f * x, f * y, th, stand]

    def send_to_foulzone(self, team, id):
        f = -1 if self.half_passed else 1
        s = 1 if team == 0 else -1
        translation = [f * constants.ROBOT_FOUL_POSTURE[id][0] * s,
                       constants.ROBOT_HEIGHT[id] / 2,
                       f * -constants.ROBOT_FOUL_POSTURE[id][1] * s]
        angle = constants.PI if self.half_passed else 0
        angle += constants.ROBOT_FOUL_POSTURE[id][2]
        angle += 0 if team == 0 else constants.PI
        angle -= constants.PI / 2
        rotation = [0, 1, 0, angle]

        node = self.robot[team][id]['node']
        al = node.getField('axleLength').getSFFloat()
        h = node.getField('height').getSFFloat()
        wr = node.getField('wheelRadius').getSFFloat()
        lwTranslation = [-al / 2, (-h + 2 * wr) / 2, 0]
        rwTranslation = [al / 2, (-h + 2 * wr) / 2, 0]
        wheelRotation = [1, 0, 0, constants.PI / 2]
        node.getField('translation').setSFVec3f(translation)
        node.getField('rotation').setSFRotation(rotation)
        node.getField('customData').setSFString('0 0')
        node.getField('lwTranslation').setSFVec3f(lwTranslation)
        node.getField('lwRotation').setSFRotation(wheelRotation)
        node.getField('rwTranslation').setSFVec3f(rwTranslation)
        node.getField('rwRotation').setSFRotation(wheelRotation)
        node.resetPhysics()

    def run(self):
        config_file = open('../../config.json')
        config = json.loads(config_file.read())
        self.game_time = constants.DEFAULT_GAME_TIME_MS / constants.PERIOD_MS * constants.PERIOD_MS
        deadlock_flag = True
        if config['rule']:
            if config['rule']['game_time']:
                self.game_time = config['rule']['game_time'] * 1000 / constants.PERIOD_MS * constants.PERIOD_MS
            if config['rule']['deadlock']:
                deadlock_flag = config['rule']['deadlock']
        else:
            print('"rule" section of \'config.json\' seems to be missing: using default options\n')
        print('Rules:\n')
        print('     game duration - ' + str(self.game_time / 1000) + ' seconds\n')
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
        self.ready[0] = False
        self.ready[1] = False
        # gets the teams' information from 'config.json'
        for team in [0, 1]:
            if team == 0:
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
            player_team_infos.append([name, rating, path_prefix + exe, path_prefix, 'ROLE_PLAYER', team == 0])
            if team == 0:
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
            info['game_time'] = self.game_time / 1000
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
        self.ball = self.getFromDef(constants.DEF_BALL)
        self.time = 0
        self.kickoff_time = self.time
        self.score = [0, 0]
        self.half_passed = False
        self.reset_reason = Game.GAME_START
        self.game_state = Game.STATE_KICKOFF
        self.ball_ownership = 0  # red
        self.robot = [[0 for x in range(constants.NUMBER_OF_ROBOTS)] for y in range(2)]
        for t in range(2):
            for id in range(constants.NUMBER_OF_ROBOTS):
                node = self.getFromDef(robot_name(t, id))
                self.robot[t][id] = {}
                self.robot[t][id]['node'] = node
                position = node.getPosition()
                self.robot[t][id]['x'] = position[0]
                self.robot[t][id]['y'] = position[2]
                orientation = node.getOrientation()
                self.robot[t][id]['th'] = orientation[3]
                self.robot[t][id]['active'] = True
                self.robot[t][id]['touch'] = False
        self.reset(constants.ROBOT_KICKOFF, constants.ROBOT_DEFAULT)
        self.lock_all_robots(True)
        self.robot[0][4]['active'] = True

        # start participants
        for player_team_info in player_team_infos:
            exe = player_team_info[2]
            color = 0 if player_team_info[5] else 1
            if not os.path.exists(exe):
                print('Participant controller not found: ' + exe)
            else:
                command_line = []
                if exe.endswith('.py'):
                    os.environ['PYTHONPATH'] += os.pathsep + os.path.join(os.getcwd(), 'player_py')
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
        while True:
            self.update_label()
            sys.stdout.flush()
            self.tcp_server.spin(self)
            if not self.started:
                if self.ready[0] and self.ready[1]:
                    print('Starting match.')
                    self.started = True
                else:
                    if self.step(self.timeStep) == -1:
                        break
                    continue
            self.update_positions()
            if self.time > self.game_time:  # half of game over
                if self.half_passed:  # game over
                    if self.repeat:
                        self.game_state = Game.EPISODE_END
                        self.episode_restart()
                    else:
                        self.game_state = Game.GAME_END
                else:  # second half starts with a kickoff by the blue team (1)
                    self.game_state = Game.HALFTIME
                    self.mark_half_passed()
                    self.ball_ownership = 1
                    self.game_state = Game.KICKOFF
                    self.kickoff_time = self.time
                    self.reset(constants.ROBOT_DEFAULT, constants.ROBOT_KICKOFF)
                    self.lock_all_robots()
                    self.robot[1][4]['active'] = True
                self.half_passed = not self.half_passed
                self.stop_robots()
                self.step(constants.WAIT_END_MS)
                self.time = 0
                self.reset_reason = constants.GAME_START
            for team in range(2):
                frame = self.generate_frame(team)
                self.tcp_server.send(self.team_client[team], json.dumps(frame))
            # if any of the robots has touched the ball at this frame, update self.recent_touch
            touch = self.get_robot_touch_ball()
            for team in range(2):
                for id in range(constants.NUMBER_OF_ROBOTS):
                    if touch[team][id]:
                        self.recent_touch = touch
                        break
            # check if any of robots has fallen
            for team in range(2):
                for id in range(constants.NUMBER_OF_ROBOTS):
                    # if a robot has fallen and could not recover for c::FALL_TIME_MS, send the robot to foulzone
                    if self.robot[team][id]['active'] \
                       and self.time - self.robot[team][id]['fall_time'] >= constants.FALL_TIME_MS:
                        self.robot[team][id]['active'] = False
                        self.send_to_foulzone(team, id)
                        self.robot[team][id]['sentout_time'] = self.time
                    elif self.get_robot_posture(team, id)[3]:  # robot is standing properly
                        self.robot[team][id]['fall_time'] = self.time

            if self.game_state == Game.STATE_DEFAULT:
                ball_x = self.ball_position[0]
                ball_y = self.ball_position[2]
                if abs(ball_x) > constants.FIELD_LENGTH / 2 and abs(ball_y) < constants.GOAL_WIDTH / 2:
                    goaler = 0 if ball_x > 0 else 1
                    self.score[goaler] += 1
                    self.update_label()
                    self.stop_robots()
                    self.step(constants.WAIT_GOAL_MS)
                    self.game_state = Game.STATE_KICKOFF
                    self.ball_ownership = 1 if ball_x > 0 else 0
                    self.kickoff_time = self.time
                    self.reset(constants.ROBOT_KICKOFF if self.ball_ownership == 0 else constants.ROBOT_DEFAULT,
                               constants.ROBOT_KICKOFF if self.ball_ownership == 1 else constants.ROBOT_DEFAULT)
                    self.lock_all_robots(True)
                    self.robot[0 if self.ball_ownership == 0 else 1][4]['active'] = True
                    self.step(constants.WAIT_STABLE_MS)
                    self.reset_reason = constants.SCORE_RED_TEAM if ball_x > 0 else constants.SCORE_BLUE_TEAM

            elif self.game_state == Game.STATE_KICKOFF:
                if self.time - self.kickoff_time >= constants.KICKOFF_TIME_LIMIT_MS:
                    self.game_state = Game.STATE_DEFAULT
                    self.lock_all_robots(False)
                else:
                    ball_x = self.ball_position[0]
                    ball_y = self.ball_position[2]
                    if ball_x * ball_x + ball_y * ball_y > constants.KICKOFF_BORDER * constants.KICKOFF_BORDER:
                        self.game_state = Game.STATE_DEFAULT
                        self.lock_all_robots(False)
                self.deadlock_time = self.time
            if self.step(self.timeStep) == -1:
                break
            self.time += self.timeStep


controller = GameSupervisor()
controller.run()
