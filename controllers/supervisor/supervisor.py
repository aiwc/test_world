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

        tcp_server = TcpServer(constants.SERVER_IP, constants.SERVER_PORT)
        while True:
            tcp_server.spin()
            if self.step(self.timeStep) == -1:
                break


controller = GameSupervisor()
controller.run()
