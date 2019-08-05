#include "player.hpp"

#include <arpa/inet.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

Player::Player(char **argv) {
  int port = std::stoi(argv[2], nullptr);
  mKey = argv[3];
  mData = argv[4];

  struct sockaddr_in server_addr = {0};

  // assign IP, PORT
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(argv[1]);
  server_addr.sin_port = htons(port);

  // create the socket
  mConnFd = socket(AF_INET, SOCK_STREAM, 0);
  if (mConnFd == -1) {
    printf("socket creation failed...\n");
    exit(0);
  }

  // connect the client socket to server socket
  if (connect(mConnFd, (struct sockaddr *)&server_addr, sizeof(server_addr)) !=
      0) {
    printf("connection with the server failed...\n");
    exit(0);
  }
}

Player::~Player() {
  if (shutdown(mConnFd, SHUT_RDWR) == -1 || close(mConnFd) == -1)
    printf("Failed to shutdown connection with the server...\n");
}

void Player::sendToServer(std::string message, std::string arguments) {
  std::string toSend = "aiwc." + message + "(\"" + mKey + "\"";
  if (arguments.size() > 0)
    toSend += ", " + arguments;
  toSend += ")";
  const char *toSendString = toSend.c_str();
  send(mConnFd, (void *)toSendString, strlen(toSendString) * sizeof(char), 0);
}

json Player::receive() {
  char buffer[4096];
  memset(buffer, '\0', sizeof(buffer));
  int ret = read(mConnFd, (void *)buffer, sizeof(buffer) - 1);
  return json::parse(buffer);
}

void Player::setSpeeds(std::vector<double> speeds) {
  std::string arguments = "";
  for (unsigned i = 0; i < speeds.size(); i++)
    arguments += std::to_string(speeds[i]) + ", ";
  arguments = arguments.substr(0, arguments.size() - 2);
  sendToServer("set_speeds", arguments);
}

bool Player::check_frame(json frame) {
  if (frame.find("reset_reason") != frame.end() &&
      frame["reset_reason"] == 4) // TODO: 4 = Game.GAME_END
    return false;
  return true;
}

void Player::init(json info) { printf("init() method called...\n"); }

void Player::update(json frame) { printf("update() method called...\n"); }

void Player::finish() { printf("finish() method called...\n"); }

void Player::run() {
  sendToServer("get_info");
  init(receive());
  sendToServer("ready");

  while (true) {
    json frame = receive();
    if (!frame.empty()) {
      if (check_frame(frame)) // return false if we need to quit
        update(frame);
      else
        break;
    } else
      break;
  }

  finish();
  return;
}
