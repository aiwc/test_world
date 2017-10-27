#include <webots/DifferentialWheels.hpp>

#include <sstream>
#include <string>

#include <cassert>

class soccer_robot
  : public webots::DifferentialWheels
{
public:
  soccer_robot()
  { }

  void run()
  {
    while(step(1) != -1) {
      const double max_speed = getMaxSpeed();

      std::stringstream ss(getData());
      double left, right;
      ss >> left >> right;

      left  = std::min(std::max(left,  -max_speed), max_speed);
      right = std::min(std::max(right, -max_speed), max_speed);

      setSpeed(left, right);
    }
  }
};

int main()
{
  soccer_robot sr;
  sr.run();
  return 0;
}
