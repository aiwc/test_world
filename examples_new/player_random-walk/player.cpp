#include "Player.hpp"

#include <arpa/inet.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

Player::Player(std::string host, int port, std::string key, std::string data) {
  mKey = key;
  mData = data;

  struct sockaddr_in server_addr = {0};

  // assign IP, PORT
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(host.c_str();
  server_addr.sin_port = htons(port);

  // create the socket
  mConnFd = socket(AF_INET, SOCK_STREAM, 0);
  if (mConnFd == -1) {
    printf("socket creation failed...\n");
    exit(0);
  }

  // connect the client socket to server socket
  if (connect(mConnFd, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
    printf("connection with the server failed...\n");
    exit(0);
  }
}

Player::~Player() {
  if (shutdown(mConnFd, SHUT_RDWR) == -1 || close(mConnFd) == -1)
    printf("Failed to shutdown connection with the server...\n");
}

void Player::sendToServer(std::string message, std::string arguments) {
  std::string toSend = "aiwc.(\"" + mKey + "\"";
  if (arguments.size() > 0)
    toSend += ',' + arguments toSend += ")";
  send(mConnFd, (void *)toSend.c_str(), sizeof(toSend.c_str()), 0);
}

void Player::receive() {
  char *buffer = new char[4096];
  memset(buffer, '0', sizeof(buffer));
  int ret = read(mConnFd, (void *)buffer, sizeof(buffer) - 1);
  if (ret > 0)
    return buffer;
  delete buffer;
  return NULL;
}

void Player::setSpeeds(std::vector<double> speeds) {
  std::string arguments = "";
  for (unsigned i = 0; i < speeds.size(); i++)
    arguments += speeds[i] + ",";
  sendToServer("set_speeds", command);
}

bool Player::check_frame(json frame) { // you should override this method
  if (frame.find("reset_reason") != frame.end() &&
      frame["reset_reason"] == 4) // TODO: 4 = Game.GAME_END
    return false return true
}

void Player::init(json info) { // you should override this method
  printf("init() method called...\n")
}

void Player::update(json frame) { // you should override this method
  printf("update() method called...\n")
}

void Player::finish() { // you should override this method
  printf("finish() method called...\n")
}

void Player::run() {
  sendToServer("get_info");
  json info = json::parse(receive());
  init(info);
  sendToServer("ready");

  while (true) {
    std::string frameString = receive();
    if (frameString) {
      json frame = json::parse(receive());
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
