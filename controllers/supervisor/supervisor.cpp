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

  for(;;) {
    game g(sv);
    if(g.run() == -1) {
      break;
    }
  }

  wr.shutdown();
  wamp_router_th.join();

  return 0;
}
