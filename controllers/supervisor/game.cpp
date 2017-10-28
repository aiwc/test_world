#include "game.hpp"

#include <boost/format.hpp>

#include <random>
#include <string>

namespace c = constants;
namespace bp = boost::process;

namespace /* anonymous */ {

  enum {
    T_RED = 0,
    T_BLUE = 1,
  };

  std::string random_string(std::size_t len)
  {
    constexpr const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    constexpr std::size_t alphanum_size = sizeof(alphanum) - 1; // -1 to exclude terminating null

    std::random_device rd{};
    std::default_random_engine re{rd()};
    std::uniform_int_distribution<std::size_t> uid(0, alphanum_size - 1);

    std::string ret;
    std::generate_n(std::back_inserter(ret), len, [&]() { return alphanum[uid(re)]; });
    return ret;
  }

} // namespace /* anonymous */


// converters for types
namespace msgpack {
  MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
    namespace adaptor {

      template <>
      struct object_with_zone<subimage> {
        void operator()(msgpack::object::with_zone& o, const subimage& v) const {
          o.type = type::MAP;
          o.via.map.size = 5;
          o.via.map.ptr = static_cast<msgpack::object_kv*>(o.zone.allocate_align(sizeof(msgpack::object_kv) * o.via.map.size));
          o.via.map.ptr[0] = { msgpack::object("x", o.zone), msgpack::object(v.x, o.zone) };
          o.via.map.ptr[1] = { msgpack::object("y", o.zone), msgpack::object(v.y, o.zone) };
          o.via.map.ptr[2] = { msgpack::object("w", o.zone), msgpack::object(v.w, o.zone) };
          o.via.map.ptr[3] = { msgpack::object("h", o.zone), msgpack::object(v.h, o.zone) };
          o.via.map.ptr[4] = { msgpack::object("base64", o.zone), msgpack::object(v.base64, o.zone) };
        }
      };

    } // namespace adaptor
  } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

game::game(supervisor& sv)
  : sv_(sv)
{ }

int game::run()
{
  if(work_) {
    throw std::runtime_error("Game already started");
  }

  int ret = 0;
  events_stop_ = false;

  // launch io thread
  work_ = std::make_unique<boost::asio::io_service::work>(io_);
  io_thread_ = std::thread([&]() { io_.run(); });

  // laucnh publish thread
  publish_thread_ = std::thread([&]() noexcept {
      for(;;) {
        std::unique_lock<std::mutex> lck(events_mutex_);
        events_cv_.wait(lck, [&]() { return events_stop_ || !events_.empty(); });

        auto local_events = std::move(events_);
        lck.unlock();

        while(!local_events.empty()) {
          const auto& front = local_events.front();
          const auto& topic = std::get<0>(front);
          const auto& args  = std::get<1>(front);

          // we have to wait until completely published to keep zone alive
          session_->publish(topic, args).get();
          local_events.pop_front();
        }

        if(events_stop_) {
          break;
        }
      }
    });

  // gets the teams' information from the matching server
  // In test code we use static information
  for(const auto& team : {T_RED, T_BLUE}) {
    const auto tup = sv_.get_team_info(team == T_RED);
    const auto tup_op = sv_.get_team_info(team != T_RED);

    // my team
    const std::string& name   = std::get<0>(tup);
    const double&      rating = std::get<1>(tup);
    const std::string& exe    = std::get<2>(tup);
    const std::string& data   = std::get<3>(tup);

    // opponent
    const std::string& name_op   = std::get<0>(tup_op);
    const double&      rating_op = std::get<1>(tup_op);

    const auto ret = player_team_infos_.emplace(random_string(c::KEY_LENGTH),
                                                team_info(name, rating, exe, data,
                                                          team == T_RED,
                                                          1000 * c::FOUL_DURATION / c::PERIOD_MS
                                                          ));

    assert(ret.second);
    auto& ti = ret.first->second;

    // create information for aiwc.get_info() in advance
    using map = msgpack::type::assoc_vector<std::string, msgpack::object>;
    map info;
    info.emplace_back("field",        msgpack::object(std::make_tuple(c::FIELD_LENGTH, c::FIELD_WIDTH), ti.z_info));
    info.emplace_back("goal",         msgpack::object(std::make_tuple(c::GOAL_DEPTH, c::GOAL_WIDTH), ti.z_info));
    info.emplace_back("penalty_area", msgpack::object(std::make_tuple(c::PENALTY_AREA_DEPTH,
                                                                      c::PENALTY_AREA_WIDTH), ti.z_info));
    info.emplace_back("ball_radius",          msgpack::object(c::BALL_RADIUS, ti.z_info));
    info.emplace_back("robot_size",           msgpack::object(c::ROBOT_SIZE, ti.z_info));
    info.emplace_back("axle_length",          msgpack::object(c::AXLE_LENGTH, ti.z_info));
    info.emplace_back("max_linear_velocity",  msgpack::object(c::MAX_LINEAR_VELOCITY, ti.z_info));

    info.emplace_back("resolution", msgpack::object(std::make_tuple(c::RESOLUTION_X, c::RESOLUTION_Y), ti.z_info));
    info.emplace_back("number_of_robots", msgpack::object(c::NUMBER_OF_ROBOTS, ti.z_info));
    info.emplace_back("codewords",  msgpack::object(c::CODEWORDS, ti.z_info));
    info.emplace_back("game_time",   msgpack::object(c::GAME_TIME, ti.z_info));

    info.emplace_back("team_info",
                      msgpack::object(std::make_tuple(map{std::make_pair("name",   msgpack::object(name, ti.z_info)),
                              /* */                       std::make_pair("rating", msgpack::object(rating, ti.z_info))},
                          /* */                       map{std::make_pair("name",   msgpack::object(name_op, ti.z_info)),
                              /* */                       std::make_pair("rating", msgpack::object(rating_op, ti.z_info))}),
                        /* */         ti.z_info));

    ti.info = msgpack::object(info, ti.z_info);
  }

  // initialize promises and futures
  bootup_promise_ = {};
  ready_promise_ = {};
  auto bootup_future = bootup_promise_.get_future();
  auto ready_future = ready_promise_.get_future();

  connect_to_server();

  // bootup VMs & wait until 2 players boot up
  bootup_vm();
  bootup_future.wait();

  // wait until 2 players are ready for c::WAIT_READY seconds
  ready_future.wait_until(std::chrono::steady_clock::now()
                          + std::chrono::seconds(c::WAIT_READY));

  // run or finish the game
  {
    std::unique_lock<std::mutex> lck(player_team_infos_mutex_);
    const auto count = std::count_if(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                                     [](const auto& kv) { return kv.second.is_ready == true; });
    if(count == 0) {
      // send the result(draw) to the matching server
    }
    else if(count == 1) {
      const auto it = std::find_if(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                                   [](const auto& kv) { return kv.second.is_ready == true; });
      assert(it != std::cend(player_team_infos_));

      // TODO: send winning frame to team pointed by it.
      ret = 0;
    }
    else {
      lck.unlock();

      try {
        std::cout << "Starting a new game" << std::endl;
        run_game();
      }
      catch(const webots_revert_exception& e) {
        ret = -1;
      }
    }
  }

  if(ret == -1) { // if webots reverts
    terminate_vm();
  }
  else { // Not webots revert
    // now players have c::WAIT_KILL seconds to finish
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(c::WAIT_KILL);

    std::cout << "Waiting players to finish" << std::endl;

    for(auto& kv : player_team_infos_) {
      auto& ti = kv.second;

      // boost 1.65 or lower has a bug in child::wait_until(). use child::wait_for().
      // ti.c.wait_until(until);
      ti.c.wait_for(until - std::chrono::steady_clock::now());
      if(ti.c.running()) {
        ti.c.terminate();
      }
    }
  }

  // stop publishing and wait until publish thread stops
  events_stop_ = true;
  events_cv_.notify_one();
  publish_thread_.join();

  session_->leave().get();
  session_->stop().get();
  transport_->detach();


  io_.post([&]() {
      work_.reset();
      io_.poll();
      io_.stop();
    });

  io_thread_.join();

  return ret;
}

void game::connect_to_server()
{

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
  boost::asio::local::stream_protocol::endpoint uds_endpoint(c::RS_PATH);
  transport_ = std::make_shared<autobahn::wamp_uds_transport>(io_, uds_endpoint);
#else
  boost::asio::ip::tcp::endpoint tcp_endpoint(boost::asio::ip::address::from_string(constants::SERVER_IP),
                                              constants::RS_PORT);
  transport_ = std::make_shared<autobahn::wamp_tcp_transport>(io_, tcp_endpoint);
#endif
  session_   = std::make_shared<autobahn::wamp_session>(io_);
  transport_->attach(std::static_pointer_cast<autobahn::wamp_transport_handler>(session_));

  transport_->connect().get();
  session_->start().get();
  session_->join(constants::REALM).get();

  // register calles
  session_->provide("aiwc.bootup",    [&](autobahn::wamp_invocation i) { return on_bootup(std::move(i)); }).get();
  session_->provide("aiwc.ready",     [&](autobahn::wamp_invocation i) { return on_ready(std::move(i)); }).get();
  session_->provide("aiwc.get_info",  [&](autobahn::wamp_invocation i) { return on_info(std::move(i)); }).get();
  session_->provide("aiwc.set_speed", [&](autobahn::wamp_invocation i) { return on_set_speed(std::move(i)); }).get();
}

void game::bootup_vm()
{
  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);

  // bootup VMs. replaced to process.
  for(auto& kv : player_team_infos_) {
    const auto& key = kv.first;
    auto& ti = kv.second;

    // send bootup signal. VM's startup script is supposed to do this.
    session_->call("aiwc.bootup", std::make_tuple(key));

    // launch process
    boost::filesystem::path p_exe = ti.executable;

    ti.c = bp::child(bp::exe = ti.executable,
                     bp::args = {c::SERVER_IP,
                         std::to_string(c::RS_PORT),
                         c::REALM,
                         key,
                         boost::filesystem::absolute(ti.datapath).string()},
                     bp::start_dir = p_exe.parent_path());
  }
}

void game::terminate_vm()
{
  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);

  for(auto& kv : player_team_infos_) {
    kv.second.c.terminate();
  }
}

void game::update_label()
{
  sv_.setLabel(0,
               (boost::format("score %d:%d, time %.2f") % score_[0] % score_[1] % time_).str(),
               0.4, 0.95, // x, y
               0.10, 0x00000000, // size, color
               0, "Arial" // transparency, font
               );
}

void game::step(std::size_t ms)
{
  if(sv_.step(ms) == -1) {
    throw webots_revert_exception();
  }
}

void game::reset_position() {
  sv_.reset_position();

  for(auto& team_activeness : activeness_) {
    for(auto& robot_activeness : team_activeness) {
      robot_activeness = true;
    }
  }

  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);
  for(auto& kv : player_team_infos_) {
    auto& ti    = kv.second;

    // reset wheel speed to 0
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      ti.wheel_speed[id] = {0, 0};
    }

    // reset foul counter
    ti.foul_count.clear();
  }
}

void game::set_speed(bool stop_all)
{
  constexpr std::array<double, 2> stop = {0, 0};

  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);
  for(auto& kv : player_team_infos_) {
    auto& ti    = kv.second;
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      if(stop_all || !activeness_[ti.is_red ? T_RED : T_BLUE][id]) {
        sv_.set_linear_wheel_speed(ti.is_red, id, stop);
      }
      else {
        sv_.set_linear_wheel_speed(ti.is_red, id, ti.wheel_speed[id]);
      }
    }
  }
}

std::size_t game::count_robots_in_penalty_area(bool is_red) const
{
  std::size_t ret = 0;

  constexpr auto is_in_goal_area = [](double x, double y) {
    return (x < -c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH);
  };

  constexpr auto is_in_penalty_area = [](double x, double y) {
    return
    (x >= -c::FIELD_LENGTH / 2)
    && (x < -c::FIELD_LENGTH / 2 + c::PENALTY_AREA_DEPTH)
    && (std::abs(y) < c::PENALTY_AREA_WIDTH);
  };

  for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
    const auto pos = sv_.get_robot_posture(is_red, id);
    const double sign = is_red ? 1 : -1;

    const auto x = sign * std::get<0>(pos);
    const auto y = sign * std::get<1>(pos);

    if(is_in_goal_area(x, y) || is_in_penalty_area(x, y)) {
      ++ret;
    }
  }

  return ret;
}

void game::publish_current_frame(std::size_t reset_reason)
{
  // get ball and robots position
  const auto g_ball = sv_.get_ball_position();
  std::array<std::array<std::tuple<double, double, double, bool>, c::NUMBER_OF_ROBOTS>, 2> g_robots;
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      const auto r = sv_.get_robot_posture(team == T_RED, id);
      std::get<0>(g_robots[team][id]) = std::get<0>(r);
      std::get<1>(g_robots[team][id]) = std::get<1>(r);
      std::get<2>(g_robots[team][id]) = std::get<2>(r);
      std::get<3>(g_robots[team][id]) = activeness_[team][id];
    }
  }

  std::vector<std::tuple<std::string, msgpack::object, msgpack::zone> > events;

  {
    std::unique_lock<std::mutex> lck(player_team_infos_mutex_);
    for(auto& kv : player_team_infos_) {
      const auto& topic = kv.first;
      auto& ti    = kv.second;

      auto ball = g_ball;
      auto robots = g_robots;
      auto score = score_;
      auto reason = reset_reason;

      if(!ti.is_red) { // blue team sees red as blue, blue as red
        for(auto& e : ball) { e *= -1; }
        std::swap(robots[T_RED], robots[T_BLUE]);
        for(const auto& team : {T_RED, T_BLUE}) {
          for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
            std::get<0>(robots[team][id]) *= -1;
            std::get<1>(robots[team][id]) *= -1;
            std::get<2>(robots[team][id]) += c::PI;
          }
        }
        std::swap(score[T_RED], score[T_BLUE]);
        if(reason == c::SCORE_RED_TEAM) reason = c::SCORE_BLUE_TEAM;
        else if(reason == c::SCORE_BLUE_TEAM) reason = c::SCORE_RED_TEAM;
      }

      msgpack::type::assoc_vector<std::string, msgpack::object> msg;
      msgpack::zone z;

      // The first frame contains time, score, reset_reason
      msg.emplace_back("time", msgpack::object(time_, z));
      msg.emplace_back("score", msgpack::object(score, z));
      msg.emplace_back("reset_reason", msgpack::object(reason, z));

      auto subimages = ti.imbuf.update_image(sv_.get_image(ti.is_red));
      constexpr std::size_t n_max_subimages = c::MSG_MAX_SIZE / c::ESTIMATED_SUBIMAGE_SIZE;

      if(subimages.size() <= n_max_subimages) {
        msg.emplace_back("subimages", msgpack::object(subimages, z));
      }
      else {
        while(!subimages.empty()) {
          if(subimages.size() > n_max_subimages) {
            // move n_max_subimages from subimages to subs_in_msg
            std::vector<subimage> subs_in_msg;
            subs_in_msg.reserve(n_max_subimages);
            std::copy(std::make_move_iterator(subimages.begin()),
                      std::make_move_iterator(std::next(subimages.begin(), n_max_subimages)),
                      std::back_inserter(subs_in_msg));
            subimages.erase(subimages.begin(), std::next(subimages.begin(), n_max_subimages));

            msg.emplace_back("subimages", msgpack::object(subs_in_msg, z));

            events.emplace_back(topic,
                                msgpack::object(std::make_tuple(msg), z),
                                std::move(z));
            msg.clear();
            z = msgpack::zone{};
          }
          else { // last msg
            msg.emplace_back("subimages", msgpack::object(subimages, z));
            subimages.clear();
          }
        }
      }
      msg.emplace_back("EOF",          msgpack::object(true, z));
      msg.emplace_back("coordinates",
                       msgpack::object(std::make_tuple(robots[T_RED],
                                                       robots[T_BLUE],
                                                       ball), z));

      events.emplace_back(topic,
                          msgpack::object(std::make_tuple(msg), z),
                          std::move(z));
    }
  }

  std::unique_lock<std::mutex> lck_events(events_mutex_);
  std::copy(std::make_move_iterator(events.begin()),
            std::make_move_iterator(events.end()),
            std::back_inserter(events_));
  lck_events.unlock();
  events_cv_.notify_one();
}

void game::run_game()
{
  time_ = 0;
  score_ = {0, 0};

  for(auto& team_activeness : activeness_) {
    for(auto& robot_activeness : team_activeness) {
      robot_activeness = true;
    }
  }

  update_label();

  // reset and wait 1s for stabilizing
  reset_position();
  set_speed(true);
  step(c::WAIT_STABLE * 1000);

  std::size_t reset_reason = c::GAME_START;

  for(;;) {
    update_label();

    // if game ends
    if(time_ >= c::GAME_TIME) {
      publish_current_frame(c::GAME_END);
      break;
    }

    // if a team scored
    {
      const auto ball_x = std::get<0>(sv_.get_ball_position());
      if(std::abs(ball_x) > c::FIELD_LENGTH / 2) {
        ++score_[(ball_x > 0) ? T_RED : T_BLUE];
        update_label();

        // stop all and wait for c::WAIT_GOAL seconds
        set_speed(true);
        step(c::WAIT_GOAL * 1000);

        reset_position();
        step(c::WAIT_STABLE * 1000);

        reset_reason = (ball_x > 0) ? c::SCORE_RED_TEAM : c::SCORE_BLUE_TEAM;
      }
    }

    // if a team is blocking the goal area
    {
      std::unique_lock<std::mutex> lck(player_team_infos_mutex_);
      for(auto& kv : player_team_infos_) {
        auto& ti    = kv.second;
        auto& foul_counter = ti.foul_count;

        foul_counter.push_back(count_robots_in_penalty_area(ti.is_red));

        const auto sum = std::accumulate(std::cbegin(foul_counter), std::cend(foul_counter), (std::size_t)0);
        if(sum >= c::FOUL_THRESHOLD * foul_counter.capacity()) {
          std::mt19937 rng{std::random_device{}()};
          std::uniform_int_distribution<std::size_t> dist(0, 4);

          auto& team_activeness = activeness_[ti.is_red ? T_RED : T_BLUE];

          if(std::any_of(std::begin(team_activeness), std::end(team_activeness),
                         [](const auto& is_active) { return is_active; })) {
            for(;;) {
              std::size_t id = dist(rng);
              auto& is_active = activeness_[ti.is_red ? T_RED : T_BLUE][id];
              if(is_active) {
                is_active = false;
                sv_.send_to_foulzone(ti.is_red, id);
                break;
              }
            }
          }

          foul_counter.clear();
        }
      }
    }

    publish_current_frame(reset_reason);
    reset_reason = c::NONE;

    set_speed();
    step(c::PERIOD_MS);
    time_ += c::PERIOD_MS / 1000.;
  }
}

// called from io thread
void game::on_bootup(autobahn::wamp_invocation invocation)
{
  const std::string key = invocation->argument<std::string>(0);

  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);

  const auto it = player_team_infos_.find(key);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  auto& ti = it->second;
  ti.is_bootup = true;

  if(std::all_of(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                 [](const auto& kv) { return kv.second.is_bootup == true; })) {
    bootup_promise_.set_value();
  }

  invocation->empty_result();
}

// called from io thread
void game::on_ready(autobahn::wamp_invocation invocation)
{
  const std::string key = invocation->argument<std::string>(0);

  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);

  const auto it = player_team_infos_.find(key);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  auto& ti = it->second;

  if(!ti.is_ready) { // the first time it's ready
    ti.is_ready = true;

    if(std::all_of(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                   [](const auto& kv) { return kv.second.is_ready == true; })) {
      ready_promise_.set_value();
    }
  }
  else { // maybe reconnect
    ti.imbuf.reset();
  }

  invocation->empty_result();
}

// called from io thread
void game::on_info(autobahn::wamp_invocation invocation)
{
  const std::string caller = invocation->argument<std::string>(0);
  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);

  // if the caller is not a player, then error
  auto it = player_team_infos_.find(caller);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }
  auto& ti = it->second;

  invocation->result(std::make_tuple(ti.info));
}

void game::on_set_speed(autobahn::wamp_invocation invocation)
{
  const auto caller      = invocation->argument<std::string>(0);
  const auto wheel_speed = invocation->argument<std::array<double, 10> >(1);

  std::unique_lock<std::mutex> lck(player_team_infos_mutex_);

  // if the caller is not a player, then error
  auto it = player_team_infos_.find(caller);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }
  auto& ti = it->second;

  auto it_w = std::begin(wheel_speed);

  for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
    std::array<double, 2> wheel;
    wheel[0] = *it_w++;
    wheel[1] = *it_w++;
    ti.wheel_speed[id] = wheel;
  }

  invocation->empty_result();
}
