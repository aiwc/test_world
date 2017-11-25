#include "constants.hpp"
#include "supervisor.hpp"
#include "game.hpp"
#include "wamp_router.hpp"

#include <chrono>
#include <thread>

int main()
{
  using namespace constants;

  wamp_router wr(WS_PORT, RS_PORT, RS_PATH, REALM);

  auto wamp_router_th = std::thread([&]() {
      wr.run();
    });

  supervisor sv;
  game g(sv);
  g.run();

  wr.shutdown();
  wamp_router_th.join();

  sv.simulationRevert();

  return 0;
}
