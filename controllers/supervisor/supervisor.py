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
        self.config = json.loads(config_file.read())
        tcp_server = TcpServer(constants.SERVER_IP, constants.SERVER_PORT)
        while True:
            tcp_server.spin()
            if self.step(self.timeStep) == -1:
                break


controller = GameSupervisor()
controller.run()
