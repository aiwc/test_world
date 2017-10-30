#ifndef H_GAME_HPP
#define H_GAME_HPP
#pragma once

#include <boost/asio.hpp> // need to be the first header to avoid winsock error in windows

#include "supervisor.hpp"
#include "image_buffer.hpp"

#include <autobahn/autobahn.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/process.hpp>

#include <array>
#include <deque>
#include <exception>
#include <random>

namespace /* anonymous */ {

} // namespace /* anonymous */

struct webots_revert_exception
  : std::exception
{
  using base = std::exception;

  using base::base;
};

class game
{
public:
  game(supervisor& sv);

  game(const game&) = delete;
  game& operator=(const game&) = delete;
  game(game&&) = default;
  game& operator=(game&&) = default;

  // return -1 when webots tries to revert
  int run();

private:
  void connect_to_server();

  void bootup_vm();
  void terminate_vm();

  void update_label();

  // throw webots_revert_exception when webots reverts
  void step(std::size_t ms);

  // reset_position and wait for stabilizing_time_ms
  void reset_position();

  // send wheel speed to the simulator
  void set_speed(bool stop_all = false);

  std::size_t count_robots_in_penalty_area(bool is_red) const;

  void publish_current_frame(std::size_t reset_reason);

  // main runner
  void run_game();

  void on_bootup(autobahn::wamp_invocation invocation);
  void on_ready(autobahn::wamp_invocation invocation);
  void on_info(autobahn::wamp_invocation invocation);
  void on_set_speed(autobahn::wamp_invocation invocation);

private:
  supervisor& sv_;

  boost::asio::io_service io_;
  std::thread io_thread_;
  std::unique_ptr<boost::asio::io_service::work> work_;
#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
  std::shared_ptr<autobahn::wamp_uds_transport> transport_;
#else
  std::shared_ptr<autobahn::wamp_tcp_transport> transport_;
#endif
  std::shared_ptr<autobahn::wamp_session> session_;

  std::thread publish_thread_;
  std::atomic<bool> events_stop_;
  std::mutex events_mutex_;
  std::condition_variable events_cv_;
  std::deque<std::tuple<std::string, msgpack::object, msgpack::zone> > events_;

  struct team_info
  {
    team_info(std::string name, double rating, std::string exe, std::string datapath, bool is_red, std::size_t num_foul_record)
      : name(std::move(name)), rating(rating)
      , executable(std::move(exe)), datapath(std::move(datapath))
      , is_red(is_red)
      , is_bootup{false}, is_ready{false}
      , foul_count(num_foul_record)
      , imbuf(constants::RESOLUTION_X, constants::RESOLUTION_Y,
              constants::SUBIMAGE_NX, constants::SUBIMAGE_NY)
    { }

    std::string name;
    double rating;
    std::string executable;
    std::string datapath;
    boost::process::child c;

    bool is_red;
    bool is_bootup;
    bool is_ready;

    boost::circular_buffer<std::size_t> foul_count;

    msgpack::zone z_info;
    msgpack::object info; // premade aiwc.info return value

    image_buffer imbuf;
    std::array<std::array<double, 2>, constants::NUMBER_OF_ROBOTS> wheel_speed;
  };

  std::mutex player_team_infos_mutex_;
  std::map<std::string, team_info> player_team_infos_;

  double time_ = 0;
  std::array<std::size_t, 2> score_ = {{0, 0}};
  std::array<std::array<bool, constants::NUMBER_OF_ROBOTS>, 2> activeness_;

  std::promise<void> bootup_promise_;
  std::promise<void> ready_promise_;
};

#endif // H_GAME_HPP
