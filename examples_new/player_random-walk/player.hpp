#ifndef PLAYER_HPP
#define PLAYER_HPP

#include "json.hpp"

using namespace nlohmann;

class Player {

public:
  Player(std::string host, int port, std::string key, std::string data);
  virtual ~Player();

  void setSpeeds(std::vector<double> speeds);
  void run();

  virtual void init(json info);
  virtual bool check_frame(json frame);
  virtual void update(json frame);
  virtual void finish();

private:
  void sendToServer(std::string message, std::string arguments = "");
  std::string receive();

  std::string mKey;
  std::string mData;
  int mConnFd;
};

#endif
