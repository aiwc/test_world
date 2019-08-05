#include "Player.hpp"

#include <arpa/inet.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

Player::Player(std::string host, int port, std::string key, std::string data) {
  mKey = key;
  mData = data;

  int conn_fd;
  struct sockaddr_in server_addr = {0};

  // assign IP, PORT
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(host.c_str();
  server_addr.sin_port = htons(port);

  // create the socket
  conn_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (conn_fd == -1) {
    printf("socket creation failed...\n");
    exit(0);
  }

  // connect the client socket to server socket
  if (connect(conn_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
    printf("connection with the server failed...\n");
    exit(0);
  }
}

Player::~Player() {
  if (shutdown(conn_fd, SHUT_RDWR) == -1 || close(conn_fd) == -1)
    printf("Failed to shutdown connection with the server...\n");
}

void Player::send(std::string message /*, arguments=[]*/) {
  // message = 'aiwc.' + message + '("%s"' % self.key
  // for argument in arguments:
  //     if isinstance(argument, str):  # string
  //         message += ', "%s"' % argument
  //     else:  # number
  //         message += ', %s' % argument
  // message += ')'
  // self.socket.sendall(message.encode())
}

void Player::receive() {
  // data = self.socket.recv(4096)
  // return data.decode()
}

void Player::setSpeeds(speeds) { send("set_speeds", speeds); }

bool Player::check_frame(self, frame) { // you should override this method
  // if "reset_reason" in frame and frame['reset_reason'] == Game.GAME_END:
  //   return false
  return true
}

void Player::init(/*info*/) { // you should override this method
  printf("init() method called...\n")
}

void Player::update(/*frame*/) { // you should override this method
  printf("update() method called...\n")
}

void Player::finish() { // you should override this method
  printf("finish() method called...\n")
}

void Player::run() {
  send("get_info");
  info = receive();
  // init(json.loads(info))
  send("ready");

  do {
    frame = receive();
    if (frame &&
        check_frame(/*json.loads(frame)*/)) // return false if we need to quit
      update(/*json.loads(frame)*/);
    else {
      finish();
      return;
    }
  }
}
