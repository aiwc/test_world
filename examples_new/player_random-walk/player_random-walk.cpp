#include "Player.hpp"

class RandomWalkPlayer : public Player {

public:
  RandomWalkPlayer(std::string host, int port, std::string key,
                   std::string data)
      : Player(host, port, key, data) {}
  virtual ~RandomWalkPlayer() {}

  void init(std::string info) override {
    // self.number_of_robots = info['number_of_robots']
    // self.max_linear_velocity = info['max_linear_velocity']
  }

  virtual void update(/*frame*/) override {
    // speeds = []
    // for i in range(self.number_of_robots):
    //   speeds.append(random.uniform(-self.max_linear_velocity[i],
    //   self.max_linear_velocity[i]))
    //   speeds.append(random.uniform(-self.max_linear_velocity[i],
    //   self.max_linear_velocity[i]))
    // self.set_speeds(speeds)
  }
};

int main(int argc, char **argv) {
  RandomWalkPlayer *player = new RandomWalkPlayer();
  player->run();
  delete player;
  return 0;
}
