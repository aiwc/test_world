#ifndef PLAYER_HPP
#define PLAYER_HPP

#include "json.hpp"

using namespace nlohmann;

class Player {

public:
  Player(char **argv);
  virtual ~Player();

  void setSpeeds(std::vector<double> speeds);
  void run();

  // These methods should be overrriden
  virtual void init(json info);
  virtual bool check_frame(json frame);
  virtual void update(json frame);
  virtual void finish();

private:
  void sendToServer(std::string message, std::string arguments = "");
  json receive();

  std::string mKey;
  std::string mData;
  int mConnFd;
};

#endif
