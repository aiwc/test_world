#ifndef PLAYER_HPP
#define PLAYER_HPP

class Player {

public:
  Player(std::string host, int port, std::string key, std::string data);
  virtual ~Player();

  void setSpeeds(std::vector speeds);
  void run();

  virtual void init(std::string info);
  virtual bool check_frame(/*frame*/);
  virtual void update(/*frame*/);
  virtual void finish();

private:
  void sendToServer(std::string message, std::string arguments = "");
  void receive();

  std::string mKey;
  std::string mData;
  int mConnFd;
};

#endif
