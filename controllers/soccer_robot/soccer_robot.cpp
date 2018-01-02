#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <limits>

#include <sstream>
#include <string>

#include <cassert>

class soccer_robot
  : public webots::Robot
{
public:
  soccer_robot()
  {
    lw = getMotor("left wheel motor");
    rw = getMotor("right wheel motor");
  }

  void run()
  {
    while(step(1) != -1) {
      const double max_speed = lw->getMaxVelocity();

      std::stringstream ss(getCustomData());
      double left, right;
      ss >> left >> right;

      left  = std::min(std::max(left,  -max_speed), max_speed);
      right = std::min(std::max(right, -max_speed), max_speed);

      setSpeed(left, right);
    }
  }

private:
  webots::Motor *lw, *rw;

  void setSpeed(double left, double right)
  {
    lw->setPosition(std::numeric_limits<double>::infinity());
    lw->setVelocity(left);

    rw->setPosition(std::numeric_limits<double>::infinity());
    rw->setVelocity(right);
  }
};

int main()
{
  soccer_robot sr;
  sr.run();
  return 0;
}
