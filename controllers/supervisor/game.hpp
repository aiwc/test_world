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
  enum state_t {
    STATE_WAITING_BOOTUP = 0,
    STATE_WAITING_READY = 1,
    STATE_STARTED = 2,
  };

  enum role_t {
    ROLE_PLAYER = 0,
    ROLE_COMMENTATOR = 1,
    ROLE_REPORTER = 2,
  };

public:
  game(supervisor& sv);

  game(const game&) = delete;
  game& operator=(const game&) = delete;
  game(game&&) = default;
  game& operator=(game&&) = default;

  void run(); // throws webots_revert_exceptions 

private:
  void connect_to_server();

  void bootup_vm();
  void terminate_vm();

  void update_label();

  // game state control functions
  void step(std::size_t ms); // throw webots_revert_exception when webots reverts
  void pause();
  void reset();
  void resume();
  void stop_robots();

  // simulator-related functions
  void send_speed(); // send wheel speed to the simulator

  std::size_t count_robots_in_penalty_area(bool is_red) const;

  void publish_current_frame(std::size_t reset_reason);

  // main runner
  void run_game();

  void on_bootup(autobahn::wamp_invocation invocation);
  void on_ready(autobahn::wamp_invocation invocation);
  void on_info(autobahn::wamp_invocation invocation);
  void on_set_speed(autobahn::wamp_invocation invocation);
  void on_commentate(autobahn::wamp_invocation invocation);
  void on_report(autobahn::wamp_invocation invocation);

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
    team_info(std::string name, double rating, std::string exe, std::string datapath,
              const role_t& role, bool is_red)
              // std::size_t num_foul_record)
      : name(std::move(name)), rating(rating)
      , executable(std::move(exe)), datapath(std::move(datapath))
      , role(role), is_red(is_red)
      , is_bootup{false}, is_ready{false}
      , imbuf(constants::RESOLUTION_X, constants::RESOLUTION_Y,
              constants::SUBIMAGE_NX, constants::SUBIMAGE_NY)
    { }

    const std::string name;
    const double rating;
    const std::string executable;
    const std::string datapath;
    const role_t role;
    const bool is_red;

    boost::process::child c;

    bool is_bootup;
    bool is_ready;

    std::mutex m; // for shared members
    image_buffer imbuf;
  };

  std::map<std::string, team_info> player_team_infos_;

  msgpack::zone z_info_;
  msgpack::object info_[2]; // for red team's view and blue team's view

  std::atomic<state_t> state_{STATE_WAITING_BOOTUP};

  const std::size_t game_time_ms_;
  std::size_t time_ms_ = 0;
  std::array<std::size_t, 2> score_ = {{0, 0}};
  std::array<std::array<bool, constants::NUMBER_OF_ROBOTS>, 2> activeness_;
  std::atomic<bool> paused_{true};

  std::vector<autobahn::wamp_invocation> bootup_waiting_list_;

  std::array<boost::circular_buffer<std::size_t>, 2> foul_counter_;
  std::size_t deadlock_time_ = 0;

  using wheel_speed_t = std::array<std::array<std::array<double, 2>, constants::NUMBER_OF_ROBOTS>, 2>;

  wheel_speed_t io_thread_wheel_speed_ = {};          // only used in io_thread
  aiwc::spsc_buffer<wheel_speed_t> wheel_speed_ = {}; // producer: io thread, consumer: game thread

  std::mutex m_comments_;
  boost::circular_buffer<std::string> comments_{constants::NUM_COMMENTS};

  std::promise<void> bootup_promise_;
  std::promise<void> ready_promise_;
};

#endif // H_GAME_HPP
