// File:              supervisor.cpp
// Date:              Jan. 23, 2018
// Description:       AI World Cup supervisor
// Author(s):         Inbae Jeong
// Current Developer: Chansol Hong (cshong@rit.kaist.ac.kr)

#include "constants.hpp"
#include "supervisor.hpp"
#include "game.hpp"
#include "wamp_router.hpp"

#include <chrono>
#include <thread>

int main()
{
  using namespace constants;

  wamp_router wr(RS_PORT, REALM);

  auto wamp_router_th = std::thread([&]() {
      wr.run();
    });

  supervisor sv;
  game g(sv, wr.get_rs_port(), wr.get_uds_path());
  g.run();

  wr.shutdown();
  wamp_router_th.join();

  sv.simulationRevert();

  return 0;
}
