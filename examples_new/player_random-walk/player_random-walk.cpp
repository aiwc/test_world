#include "Player.hpp"

class RandomWalkPlayer : public Player {

public:
  RandomWalkPlayer(std::string host, int port, std::string key,
                   std::string data)
      : Player(host, port, key, data) {}
  virtual ~RandomWalkPlayer() {}

  void init(json info) override {
    mNumberOfRobots = info["number_of_robots"];
    mMaxSpeed = info["max_linear_velocity"][0]; // TODO: should be a vector
  }

  void update(json frame) override {
    std::vector<double> speeds;
    for (int = 0; i < 2 * mNumberOfRobots; ++i)
      speeds.append(2.0 * mMaxSpeed * (0.5 - rand() / RAND_MAX));
    set_speeds(speeds);
  }

private:
  double mMaxSpeed;
  int mNumberOfRobots;
};

int main(int argc, char **argv) {
  RandomWalkPlayer *player = new RandomWalkPlayer();
  player->run();
  delete player;
  return 0;
}
