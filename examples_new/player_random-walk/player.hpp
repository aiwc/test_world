#ifndef PLAYER_HPP
#define PLAYER_HPP

class Player {

public:
  Player(std::string host, int port, std::string key, std::string data);
  virtual ~Player();

  void setSpeeds(speeds);
  bool check_frame(self, frame);
  void run();

  virtual void init(/*info*/);
  virtual void update(/*frame*/);
  virtual void finish();

private:
  void send(std::string message /*, arguments=[]*/);
  void receive();

  std::string mKey;
  std::string mData;
};

#endif
