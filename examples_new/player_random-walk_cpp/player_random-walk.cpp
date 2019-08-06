#include "player.hpp"

class RandomWalkPlayer : public Player {

public:
  RandomWalkPlayer(char **argv) : Player(argv) {}
  virtual ~RandomWalkPlayer() {}

  void init(json info) override {
    mNumberOfRobots = info["number_of_robots"];
    for (int i = 0; i < mNumberOfRobots; ++i)
      mMaxSpeed.push_back(info["max_linear_velocity"][i]);
  }

  void update(json frame) override {
    std::vector<double> speeds;
    for (int i = 0; i < 2 * mNumberOfRobots; ++i)
      speeds.push_back(2.0 * mMaxSpeed[i / 2] *
                       (0.5 - (double)rand() / RAND_MAX));
    setSpeeds(speeds);
  }

private:
  std::vector<double> mMaxSpeed;
  int mNumberOfRobots;
};

int main(int argc, char **argv) {
  RandomWalkPlayer *player = new RandomWalkPlayer(argv);
  player->run();
  delete player;
  return 0;
}
