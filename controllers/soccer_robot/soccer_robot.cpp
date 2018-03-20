// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <boost/random/random_device.hpp>
#include <random>
#include <limits>

#include <sstream>
#include <string>

#include <cstdlib>
#include <cassert>

class soccer_robot
  : public webots::Robot
{
public:
  soccer_robot(double _sn): sn(_sn)
  {
    lw = getMotor("left wheel motor");
    rw = getMotor("right wheel motor");
  }

  void run()
  {
    while(step(1) != -1) {
      //ignore slip noise overshoot portion
      const double max_speed = lw->getMaxVelocity()/(1 + sn);

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
  double sn;

  void setSpeed(double left, double right)
  {
    lw->setPosition(std::numeric_limits<double>::infinity());
    lw->setVelocity(slipNoise(left));

    rw->setPosition(std::numeric_limits<double>::infinity());
    rw->setVelocity(slipNoise(right));
  }

  // Add slip noise
  double slipNoise(double v)
  {
    boost::random_device rd{};
    std::default_random_engine re{rd()};
    std::uniform_real_distribution<double> urd(-sn, sn);
    return v*(1 + urd(re));
  }
};

int main(int argc, char **argv)
{
  double sn = 0.0;
  if (argc > 1)
    sn = atof(argv[1]);

  soccer_robot sr(sn);
  sr.run();
  return 0;
}
