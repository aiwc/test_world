// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "game.hpp"

#include <boost/format.hpp>
#include <boost/random/random_device.hpp>

#include <random>
#include <string>

#include "rapidjson/document.h"
#include <fstream>
#include <time.h>

namespace c = constants;
namespace bp = boost::process;

#define _unused(x) ((void)(x))

namespace /* anonymous */ {

  enum {
    T_RED = 0,
    T_BLUE = 1,
  };

  std::string random_string(std::size_t len)
  {
    constexpr const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    constexpr std::size_t alphanum_size = sizeof(alphanum) - 1; // -1 to exclude terminating null

    boost::random_device rd{};
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

game::game(supervisor& sv, std::size_t rs_port, std::string uds_path)
  : sv_(sv)
  , rs_port_(rs_port)
  , uds_path_(uds_path)
{}

void game::run()
{
  if(work_) {
    throw std::runtime_error("Game already started");
  }

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

  //open 'config.json' to read configurations
  std::ifstream config_file("../../config.json");
  if (!config_file)
    throw std::runtime_error("Could not read 'config.json' configuration file.");

  std::string buffer((std::istreambuf_iterator<char>(config_file)), std::istreambuf_iterator<char>());
  config_file.close();

  rapidjson::Document config_json;
  config_json.Parse(buffer.c_str());

  if (!config_json.IsObject())
    throw std::runtime_error("Format of 'config.json' seems to be incorrect.");

  // gets game rules from 'config.json' (if no rules specified, default options are given)
  {
    game_time_ms_ = c::DEFAULT_GAME_TIME_MS / c::PERIOD_MS * c::PERIOD_MS;
    deadlock_flag_ = true;

    if (config_json.HasMember("rule") && config_json["rule"].IsObject()) { //set rules
      if (config_json["rule"].HasMember("game_time") && config_json["rule"]["game_time"].IsNumber())
        game_time_ms_ = static_cast<size_t>(config_json["rule"]["game_time"].GetDouble() * 1000) / c::PERIOD_MS * c::PERIOD_MS;

      if (config_json["rule"].HasMember("deadlock") && config_json["rule"]["deadlock"].IsBool())
        deadlock_flag_ = config_json["rule"]["deadlock"].GetBool();
    }
    else
      std::cout << "\"rule\" section of 'config.json' seems to be missing: using default options" << std::endl;

    std::cout << "Rules:" << std::endl;
    std::cout << "     game duration - " << game_time_ms_ / 1000.0 << " seconds" << std::endl;
    std::cout << "          deadlock - " << (deadlock_flag_ ? "on" : "off") << std::endl;
  }

  // gets other options from 'config.json' (if no option is specified, default option is given)
  {
    // automatic recording of the game (default: false)
    record = false;
    record_path = "";

    // automatic repetition of the game (default: false)
    repeat = false;

    if (config_json.HasMember("tool") && config_json["tool"].IsObject()) { //set other options if specified
      if (config_json["tool"].HasMember("repeat") && config_json["tool"]["repeat"].IsBool())
        repeat = config_json["tool"]["repeat"].GetBool();

      // if repeat is enabled, record is forced to be disabled
      if (repeat)
        std::cout << "Game repetition is enabled that the game recording will be disabled." << std::endl;
      else {
        if (config_json["tool"].HasMember("record") && config_json["tool"]["record"].IsBool())
          record = config_json["tool"]["record"].GetBool();

        if (record && config_json["tool"].HasMember("record_path") && config_json["tool"]["record_path"].IsString())
          record_path = config_json["tool"]["record_path"].GetString();
      }
    }
  }

  const auto path_prefix = std::string("../../");

  // gets the teams' information from 'config.json'
  for(const auto& team : {T_RED, T_BLUE}) {
    const auto tc = ((team == T_RED) ? "team_a" : "team_b");
    const auto tc_op = ((team != T_RED) ? "team_a" : "team_b");

    // my team
    const std::string& name   = ((config_json[tc].HasMember("name") && config_json[tc]["name"].IsString()) ? config_json[tc]["name"].GetString() : "");
    const double&      rating = 0; //rating is disabled
    const std::string& exe    = ((config_json[tc].HasMember("executable") && config_json[tc]["executable"].IsString()) ? config_json[tc]["executable"].GetString() : "");
    const std::string& data   = ((config_json[tc].HasMember("datapath") && config_json[tc]["datapath"].IsString()) ? config_json[tc]["datapath"].GetString() : "");

    // opponent
    const std::string& name_op   = ((config_json[tc_op].HasMember("name") && config_json[tc_op]["name"].IsString()) ? config_json[tc_op]["name"].GetString() : "");
    const double&      rating_op = 0; //rating is disabled

    const auto ret = player_team_infos_.emplace(std::piecewise_construct,
                                                std::make_tuple(random_string(c::KEY_LENGTH)),
                                                std::make_tuple(name, rating, path_prefix + exe, path_prefix + data,
                                                                ROLE_PLAYER, team == T_RED)
                                                );

    assert(ret.second);
    _unused(ret);

    std::cout << ((team == T_RED) ? "Team A: " : "Team B: ") << std::endl;
    std::cout << "  team name - " << name << std::endl;
    team_name[team] = name;
    std::cout << " executable - " << exe << std::endl;
    std::cout << "  data path - " << data << std::endl << std::endl;

    // create information for aiwc.get_info() in advance
    using map = msgpack::type::assoc_vector<std::string, msgpack::object>;
    map info;
    info.emplace_back("field",        msgpack::object(std::make_tuple(c::FIELD_LENGTH, c::FIELD_WIDTH), z_info_));
    info.emplace_back("goal",         msgpack::object(std::make_tuple(c::GOAL_DEPTH, c::GOAL_WIDTH), z_info_));
    info.emplace_back("penalty_area", msgpack::object(std::make_tuple(c::PENALTY_AREA_DEPTH,
                                                                      c::PENALTY_AREA_WIDTH), z_info_));
    info.emplace_back("goal_area", msgpack::object(std::make_tuple(c::GOAL_AREA_DEPTH,
                                                                   c::GOAL_AREA_WIDTH), z_info_));

    info.emplace_back("ball_radius",          msgpack::object(c::BALL_RADIUS, z_info_));
    info.emplace_back("ball_mass",            msgpack::object(c::BALL_MASS, z_info_));

    info.emplace_back("robot_size",           msgpack::object(c::ROBOT_SIZE, z_info_));
    info.emplace_back("robot_height",         msgpack::object(c::ROBOT_HEIGHT, z_info_));
    info.emplace_back("axle_length",          msgpack::object(c::AXLE_LENGTH, z_info_));
    info.emplace_back("robot_body_mass",      msgpack::object(c::ROBOT_BODY_MASS, z_info_));

    info.emplace_back("wheel_radius",         msgpack::object(c::WHEEL_RADIUS, z_info_));
    info.emplace_back("wheel_mass",           msgpack::object(c::WHEEL_MASS, z_info_));

    info.emplace_back("max_linear_velocity",  msgpack::object(c::MAX_LINEAR_VELOCITY, z_info_));
    info.emplace_back("max_torque",           msgpack::object(c::MAX_TORQUE, z_info_));

    info.emplace_back("resolution", msgpack::object(std::make_tuple(c::RESOLUTION_X, c::RESOLUTION_Y), z_info_));
    info.emplace_back("number_of_robots", msgpack::object(c::NUMBER_OF_ROBOTS, z_info_));
    info.emplace_back("codewords",  msgpack::object(c::CODEWORDS, z_info_));
    info.emplace_back("game_time",   msgpack::object(game_time_ms_ / 1000., z_info_));


    info.emplace_back("team_info",
                      msgpack::object(std::make_tuple(map{std::make_pair("name",   msgpack::object(name, z_info_)),
                              /* */                       std::make_pair("rating", msgpack::object(rating, z_info_))},
                          /* */                       map{std::make_pair("name",   msgpack::object(name_op, z_info_)),
                              /* */                       std::make_pair("rating", msgpack::object(rating_op, z_info_))}),
                        /* */         z_info_));

    info_[team] = msgpack::object(info, z_info_);
  }

  // gets commentator information from 'config.json' (commentator is optional)
  if (config_json.HasMember("commentator") && config_json["commentator"].IsObject()) {
    const std::string& name   = ((config_json["commentator"].HasMember("name") && config_json["commentator"]["name"].IsString()) ? config_json["commentator"]["name"].GetString() : "");
    const std::string& exe    = ((config_json["commentator"].HasMember("executable") && config_json["commentator"]["executable"].IsString()) ? config_json["commentator"]["executable"].GetString() : "");
    const std::string& data   = ((config_json["commentator"].HasMember("datapath") && config_json["commentator"]["datapath"].IsString()) ? config_json["commentator"]["datapath"].GetString() : "");

    if(!exe.empty()) {
      // commentator is treated as red team with rating 0
      const auto ret = player_team_infos_.emplace(std::piecewise_construct,
                                                  std::make_tuple(random_string(c::KEY_LENGTH)),
                                                  std::make_tuple(name, 0, path_prefix + exe, path_prefix + data,
                                                                  ROLE_COMMENTATOR, true)
                                                  );

      assert(ret.second);
      _unused(ret);

      std::cout << "Commentator: " << std::endl;
      std::cout << "  team name - " << name << std::endl;
      std::cout << " executable - " << exe << std::endl;
      std::cout << "  data path - " << data << std::endl << std::endl;
    }
    else
      std::cout << "Commentator \"executable\" is missing: skipping commentator" << std::endl;
  }
  else
    std::cout << "\"commentator\" section of 'config.json' seems to be missing: skipping commentator" << std::endl;

  // gets reporter information from 'config.json' (reporter is optional)
  if (config_json.HasMember("reporter") && config_json["reporter"].IsObject()) {
    const std::string& name   = ((config_json["reporter"].HasMember("name") && config_json["reporter"]["name"].IsString()) ? config_json["reporter"]["name"].GetString() : "");
    const std::string& exe    = ((config_json["reporter"].HasMember("executable") && config_json["reporter"]["executable"].IsString()) ? config_json["reporter"]["executable"].GetString() : "");
    const std::string& data   = ((config_json["reporter"].HasMember("datapath") && config_json["reporter"]["datapath"].IsString()) ? config_json["reporter"]["datapath"].GetString() : "");

    if(!exe.empty()) {
      // reporter is treated as red team with rating 0
      const auto ret = player_team_infos_.emplace(std::piecewise_construct,
                                                  std::make_tuple(random_string(c::KEY_LENGTH)),
                                                  std::make_tuple(name, 0, path_prefix + exe, path_prefix + data,
                                                                  ROLE_REPORTER, true)
                                                  );

      assert(ret.second);
      _unused(ret);

      std::cout << "Reporter: " << std::endl;
      std::cout << "  team name - " << name << std::endl;
      std::cout << " executable - " << exe << std::endl;
      std::cout << "  data path - " << data << std::endl << std::endl;
    }
    else
      std::cout << "Reporter \"executable\" is missing: skipping reporter" << std::endl;
  }
  else
    std::cout << "\"reporter\" section of 'config.json' seems to be missing: skipping reporter" << std::endl;

  // initialize promises and futures
  bootup_promise_ = {};
  ready_promise_ = {};
  auto bootup_future = bootup_promise_.get_future();
  auto ready_future = ready_promise_.get_future();

  connect_to_server();

  // wait until app players boot up
  run_participant();
  bootup_future.wait();

  // wait until 2 players are ready for c::WAIT_READY seconds
  ready_future.wait_until(std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(c::WAIT_READY_MS));

  // run or finish the game
  {
    const auto count = std::count_if(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                                     [](const auto& kv) { return kv.second.role == ROLE_PLAYER
                                                          && kv.second.is_ready == true; });
    if(count == 0) {
      // send the result(draw) to the matching server
    }
    else if(count == 1) {
      const auto it = std::find_if(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                                   [](const auto& kv) { return kv.second.is_ready == true; });
      assert(it != std::cend(player_team_infos_));
      _unused(it);
    }
    else {
      try {
        // if recording is enabled
        if (record) {
          std::cout << "Game recording is enabled" << std::endl;
          // Get the timestamp
          time_t rawtime;
          struct tm *timeinfo;

          time(&rawtime);
          timeinfo = localtime(&rawtime);

          // Start game recording
          record_fullpath = record_path + "/[" + std::to_string(timeinfo->tm_year + 1900) + "-" + std::to_string(timeinfo->tm_mon + 1) + "-" + std::to_string(timeinfo->tm_mday) + "T" + std::to_string(timeinfo->tm_hour) + "_" + std::to_string(timeinfo->tm_min) + "_" + std::to_string(timeinfo->tm_sec) + "]" + team_name[T_RED] + "_" + team_name[T_BLUE] + ".mp4";
          sv_.movieStartRecording(record_fullpath, 1920, 1080, 0, 100, 1, false);
        }

        std::cout << "Starting a new game" << std::endl;
        if (repeat) {
          for(;;) {
            run_game();
            sv_.mark_episode_restart();
          }
        }
        else
          run_game();

        // now players have c::WAIT_KILL seconds to finish
        const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(c::WAIT_KILL_MS);

        std::cout << "Waiting players to finish" << std::endl;

        for(auto& kv : player_team_infos_) {
          auto& ti = kv.second;

          // boost 1.65 or lower has a bug in child::wait_until(). use child::wait_for().
          ti.c.wait_for(until - std::chrono::steady_clock::now());
          if(ti.c.running()) {
            ti.c.terminate();
          }
        }

        if (record) {
          // Stop game recording
          std::cout << "Saving the recorded game as: " << record_fullpath << std::endl;
          std::cout << "Please wait until the message \033[36m\"INFO: Video creation finished.\"\033[0m is shown." << std::endl;
          sv_.movieStopRecording();
        }
      }
      catch(const webots_revert_exception& e) {
        terminate_participant();
      }
    }
  }

  // save the report if anything has been written
  if (report.size() > 0) {
    std::ofstream rfile(std::string("../../reports/") + config_json["reporter"]["name"].GetString() + ".txt");
    for (auto& line : report)
      rfile << line << std::endl;
    rfile.close();
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
}

void game::connect_to_server()
{

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
  boost::asio::local::stream_protocol::endpoint uds_endpoint(uds_path_);
  transport_ = std::make_shared<autobahn::wamp_uds_transport>(io_, uds_endpoint);
#else
  boost::asio::ip::tcp::endpoint tcp_endpoint(boost::asio::ip::address::from_string(c::SERVER_IP),
                                              rs_port_);
  transport_ = std::make_shared<autobahn::wamp_tcp_transport>(io_, tcp_endpoint);
#endif
  session_   = std::make_shared<autobahn::wamp_session>(io_);
  transport_->attach(std::static_pointer_cast<autobahn::wamp_transport_handler>(session_));

  transport_->connect().get();
  session_->start().get();
  session_->join(c::REALM).get();

  // register calles
  session_->provide("aiwc.bootup",    [&](autobahn::wamp_invocation i) { return on_bootup(i); }).get();
  session_->provide("aiwc.ready",     [&](autobahn::wamp_invocation i) { return on_ready(i); }).get();
  session_->provide("aiwc.get_info",  [&](autobahn::wamp_invocation i) { return on_info(i); }).get();
  session_->provide("aiwc.set_speed", [&](autobahn::wamp_invocation i) { return on_set_speed(i); }).get();
  session_->provide("aiwc.commentate", [&](autobahn::wamp_invocation i) { return on_commentate(i); }).get();
  session_->provide("aiwc.report",    [&](autobahn::wamp_invocation i) { return on_report(i); }).get();
}

void game::run_participant()
{
  // bootup PCs for participants. currently replaced by running a child process.
  std::vector<boost::future<autobahn::wamp_call_result> > bootup_futures;
  for(const auto& kv : player_team_infos_) {
    bootup_futures.emplace_back(session_->call("aiwc.bootup", std::make_tuple(kv.first)));
  }

  // wait for all bootup
  for(auto& f : bootup_futures) {
    f.get();
  }

  try {
    for(auto& kv : player_team_infos_) {
      const auto& key = kv.first;
      auto& ti = kv.second;

      // launch participant process
      boost::filesystem::path p_exe = ti.executable;
#ifdef _WIN32
      // Windows needs an additional routine of directly calling 'python'
      // and pass the script path as an argument to run python scripts
      if (ti.executable.compare(ti.executable.length() - 3, 3, ".py") || !boost::filesystem::exists(ti.executable)) {
#endif
      ti.c = bp::child(bp::exe = p_exe.filename().string(),
                       bp::args = {c::SERVER_IP,
                           std::to_string(rs_port_),
                           c::REALM,
                           key,
                           boost::filesystem::absolute(ti.datapath).string()},
                       bp::start_dir = p_exe.parent_path());
#ifdef _WIN32
      }
      else { // if python script, enter special handler
        ti.c = bp::child(bp::exe = bp::search_path("python").string(),
                         bp::args = {p_exe.filename().string(),
                             c::SERVER_IP,
                             std::to_string(rs_port_),
                             c::REALM,
                             key,
                             boost::filesystem::absolute(ti.datapath).string()},
                         bp::start_dir = p_exe.parent_path());
      }
#endif
    }
  }
  catch(const boost::process::process_error& err) {
    for(auto& kv : player_team_infos_) {
      auto& ti = kv.second;

      if(ti.c) {
        ti.c.terminate();
      }
    }
    std::cerr << err.what() << std::endl;
    throw std::runtime_error("one of the given executables does not exist, or cannot run");
  }
}

void game::terminate_participant()
{
  for(auto& kv : player_team_infos_) {
    kv.second.c.terminate();
  }
}

void game::update_label()
{
  if(half_passed_ == false) {
    sv_.setLabel(1, "1st Half", 0.45, 0.9, 0.10, 0x00000000, 0, "Arial");
    sv_.setLabel(0,
                 (boost::format("score %d:%d, time %.2f") % score_[0] % score_[1] % (time_ms_ / 1000.)).str(),
                 0.4, 0.95, // x, y
                 0.10, 0x00000000, // size, color
                 0, "Arial" // transparency, font
                 );
  }
  else {
    sv_.setLabel(1, "2nd Half", 0.45, 0.9, 0.10, 0x00000000, 0, "Arial");
    sv_.setLabel(0,
                 (boost::format("score %d:%d, time %.2f") % score_[1] % score_[0] % ((game_time_ms_ + time_ms_) / 1000.)).str(),
                 0.4, 0.95, // x, y
                 0.10, 0x00000000, // size, color
                 0, "Arial" // transparency, font
                 );
}

  constexpr std::size_t comments_start = 2;

  std::unique_lock<std::mutex> lck(m_comments_);
  for(std::size_t i = 0; i < comments_.size(); ++i) {
    sv_.setLabel(comments_start + i,
                 comments_[i],
                 0.01, 0.01 + 0.04 * i, // x, y
                 0.08, 0x00000000, // size, color
                 0, "Arial" // transparency, font
                 );
  }
}

// game state control functions
void game::step(std::size_t ms)
{
  // we assume that world.basicTimeStep doesn't change and the given 'ms' is multiple of the basic time step
  const std::size_t basic_time_step = static_cast<std::size_t>(sv_.getBasicTimeStep());

  const auto step_throw_if_revert = [&](std::size_t ms) {
    if(sv_.step(ms) == -1) {
      throw webots_revert_exception();
    }
  };

  for(std::size_t i = 0; i < ms / basic_time_step; ++i) {
    if(paused_.load()) {
      stop_robots();
    }
    else {
      send_speed();
      time_ms_ += basic_time_step;
    }

    update_label();
    step_throw_if_revert(basic_time_step);
  }
}

void game::pause()
{
  paused_.store(true);
}

void game::reset(c::robot_formation red_formation, c::robot_formation blue_formation)
{
  sv_.reset_position(red_formation, blue_formation);

  // reset activeness
  for(auto& team_activeness : activeness_) {
    for(auto& robot_activeness : team_activeness) {
      robot_activeness = true;
    }
  }

  // reset touch
  for(auto& team_touch : touch_) {
    for(auto& robot_touch : team_touch) {
      robot_touch = false;
    }
  }
  recent_touch_ = touch_;

  // reset fall time
  for(auto& team_ft : fall_time_) {
    for(auto& robot_ft : team_ft) {
      robot_ft = time_ms_;
    }
  }

  // reset sentout time
  for(auto& team_st : sentout_time_) {
    for(auto& robot_st : team_st) {
      robot_st = 0;
    }
  }

  // reset not_in_opponent_penalty_area time
  for(auto& team_niopa : niopa_time_) {
    for(auto& robot_niopa : team_niopa) {
      robot_niopa = time_ms_;
    }
  }

  // reset goalkeeper in penalty_area time
  for(auto& robot_ipa : gk_ipa_time_) {
    robot_ipa = time_ms_;
  }

  stop_robots();

  deadlock_time_ = time_ms_;

  // flush touch packet
  sv_.flush_touch_ball();
}

void game::resume()
{
  paused_.store(false);
}

void game::stop_robots()
{
  std::promise<void> done;
  auto done_fut = done.get_future();

  // set wheel speed, defer it to io_thread and wait
  io_.post([&]() {
      io_thread_wheel_speed_ = {};
      wheel_speed_.write(io_thread_wheel_speed_);
      done.set_value();
    });
  done_fut.get();

  send_speed();
}

void game::send_speed()
{
  constexpr std::array<double, 2> stop = {0, 0};

  const auto ws = wheel_speed_.read();

  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      if(activeness_[team][id]) {
        auto speed = ws[team][id];
        speed[0] = std::max(std::min(speed[0], c::MAX_LINEAR_VELOCITY[id]), -c::MAX_LINEAR_VELOCITY[id]);
        speed[1] = std::max(std::min(speed[1], c::MAX_LINEAR_VELOCITY[id]), -c::MAX_LINEAR_VELOCITY[id]);
        sv_.set_linear_wheel_speed(team == T_RED, id, speed);
      }
      else {
        sv_.set_linear_wheel_speed(team == T_RED, id, stop);
      }
    }
  }
}

void game::lock_all_robots()
{
  for(const auto& team : {T_RED, T_BLUE})
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id)
      activeness_[team][id] = false;
}

void game::unlock_all_robots()
{
  for(const auto& team : {T_RED, T_BLUE})
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id)
      activeness_[team][id] = true;
}

void game::unlock_robot(bool team, std::size_t id)
{
  activeness_[team][id] = true;
}

bool game::get_corner_ownership()
{
  const auto ball_x = std::get<0>(sv_.get_ball_position());
  const auto ball_y = std::get<1>(sv_.get_ball_position());
  std::size_t robot_count[2] = {0, 0};
  double robot_distance[2] = {0, 0};

  const auto s_x = (ball_x > 0) ? 1 : -1;
  const auto s_y = (ball_y > 0) ? 1 : -1;

  // count the robots and distance from the ball in the corner region of concern
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      if(!activeness_[team][id])
        continue;

      const auto robot_pos = sv_.get_robot_posture(team == T_RED, id);
      const auto x = std::get<0>(robot_pos);
      const auto y = std::get<1>(robot_pos);

      // the robot is located in the corner region of concern
      if((s_x * x > c::FIELD_LENGTH / 2 - c::PENALTY_AREA_DEPTH) && (s_y * y > c::PENALTY_AREA_WIDTH / 2)) {
        const auto distance_squared = (x-ball_x)*(x-ball_x) + (y-ball_y)*(y-ball_y);
        robot_count[team] += 1;
        robot_distance[team] += sqrt(distance_squared);
      }
    }
  }

  // decision - team with more robots near the ball gets the ownership
  if(robot_count[T_RED] > robot_count[T_BLUE]) {
    return T_RED;
  }
  else if(robot_count[T_BLUE] > robot_count[T_RED]) {
    return T_BLUE;
  }
  // tie breaker - team with robots (within the decision region) closer to the ball on average gets the ownership
  else {
    if(robot_distance[T_RED] < robot_distance[T_BLUE]) {
      return T_RED;
    }
    else if(robot_distance[T_BLUE] < robot_distance[T_RED]) {
      return T_BLUE;
    }
    // a total tie - the attacker team gets an advantage
    else {
      return (ball_x > 0) ? T_RED : T_BLUE;
    }
  }
}

bool game::get_pa_ownership()
{
  const auto ball_x = std::get<0>(sv_.get_ball_position());
  const auto ball_y = std::get<1>(sv_.get_ball_position());
  std::size_t robot_count[2] = {0, 0};
  double robot_distance[2] = {0, 0};

  const auto s_x = (ball_x > 0) ? 1 : -1;

  // count the robots and distance from the ball in the penalty area of concern
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      if(!activeness_[team][id])
        continue;

      const auto robot_pos = sv_.get_robot_posture(team == T_RED, id);
      const auto x = std::get<0>(robot_pos);
      const auto y = std::get<1>(robot_pos);

      // the robot is located in the corner region of concern
      if((s_x * x > c::FIELD_LENGTH / 2 - c::PENALTY_AREA_DEPTH) && (std::abs(y) < c::PENALTY_AREA_WIDTH / 2)) {
        const auto distance_squared = (x-ball_x)*(x-ball_x) + (y-ball_y)*(y-ball_y);
        robot_count[team] += 1;
        robot_distance[team] += sqrt(distance_squared);
      }
    }
  }

  // decision - team with more robots near the ball gets the ownership
  if(robot_count[T_RED] > robot_count[T_BLUE]) {
    return T_RED;
  }
  else if(robot_count[T_BLUE] > robot_count[T_RED]) {
    return T_BLUE;
  }
  // tie breaker - team with robots (within the decision region) closer to the ball on average gets the ownership
  else {
    if(robot_distance[T_RED] < robot_distance[T_BLUE]) {
      return T_RED;
    }
    else if(robot_distance[T_BLUE] < robot_distance[T_RED]) {
      return T_BLUE;
    }
    // a total tie - the attacker team gets an advantage
    else {
      return (ball_x > 0) ? T_RED : T_BLUE;
    }
  }
}

bool game::check_penalty_area()
{
  const auto ball_x = std::get<0>(sv_.get_ball_position());
  const auto ball_y = std::get<1>(sv_.get_ball_position());
  std::size_t robot_count[2] = {0, 0};

  // check if the ball is not in the penalty area
  if((std::abs(ball_x) < c::FIELD_LENGTH / 2 - c::PENALTY_AREA_DEPTH) || (std::abs(ball_y) > c::PENALTY_AREA_WIDTH / 2)) {
    return false;
  }

  const auto s_x = (ball_x > 0) ? 1 : -1;

  // count the robots and distance from the ball in the penalty area of concern
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      if(!activeness_[team][id])
        continue;

      const auto robot_pos = sv_.get_robot_posture(team == T_RED, id);
      const auto x = std::get<0>(robot_pos);
      const auto y = std::get<1>(robot_pos);

      // the robot is located in the penalty area of concern
      if((s_x * x > c::FIELD_LENGTH / 2 - c::PENALTY_AREA_DEPTH) && (std::abs(y) < c::PENALTY_AREA_WIDTH / 2)) {
        robot_count[team] += 1;
      }
    }
  }

  if(ball_x < 0) {
    //the ball is in Team Red's penalty area
    if(robot_count[T_RED] > c::PA_THRESHOLD_D) {
      ball_ownership_ = T_BLUE;
      return true;
    }
    if(robot_count[T_BLUE] > c::PA_THRESHOLD_A) {
      ball_ownership_ = T_RED;
      return true;
    }
  }
  else {
    //the ball is in Team Blue's penalty area
    if(robot_count[T_BLUE] > c::PA_THRESHOLD_D) {
      ball_ownership_ = T_RED;
      return true;
    }
    if(robot_count[T_RED] > c::PA_THRESHOLD_A) {
      ball_ownership_ = T_BLUE;
      return true;
    }
  }
  return false;
}

bool game::robot_in_field(bool is_red, std::size_t id)
{
  const auto robot_pos = sv_.get_robot_posture(is_red, id);
  const auto x = std::get<0>(robot_pos);
  const auto y = std::get<1>(robot_pos);

  if (std::abs(y) < c::GOAL_WIDTH / 2) {
    if (std::abs(x) > c::FIELD_LENGTH / 2 + c::GOAL_DEPTH)
      return false;
    else
      return true;
  }

  if (std::abs(x) > c::FIELD_LENGTH / 2)
    return false;
  else
    return true;
}

bool game::ball_in_field()
{
  const auto pos = sv_.get_ball_position();

  // checking with absolute values is sufficient since the field is symmetrical
  const auto abs_x = std::abs(std::get<0>(pos));
  const auto abs_y = std::abs(std::get<1>(pos));

  if ((abs_x > c::FIELD_LENGTH / 2 + c::WALL_THICKNESS) && (abs_y > c::GOAL_WIDTH / 2 + c::WALL_THICKNESS))
    return false;

  if (abs_y > c::FIELD_WIDTH / 2 + c::WALL_THICKNESS)
    return false;

  // check triangular region at the corner
  const auto cs_x = c::FIELD_LENGTH / 2 - c::CORNER_LENGTH;
  const auto cs_y = c::FIELD_WIDTH / 2 + c::WALL_THICKNESS;
  const auto ce_x = c::FIELD_LENGTH / 2 + c::WALL_THICKNESS;
  const auto ce_y = c::FIELD_WIDTH / 2 - c::CORNER_LENGTH;

  if (cs_x < abs_x && abs_x < ce_x) {
    const auto border_y = ce_y + (abs_x - ce_x)*(ce_y - cs_y)/(ce_x - cs_x);
    if (abs_y > border_y)
      return false;
  }

  return true;
}

bool game::any_object_nearby(double target_x, double target_y, double target_r)
{
  // check ball position
  const auto pos = sv_.get_ball_position();

  const auto x = std::get<0>(pos);
  const auto y = std::get<1>(pos);

  const auto dist_sq = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y);

  // the ball is within the region
  if(dist_sq < target_r * target_r)
    return true;

  // check robot positions
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      const auto pos = sv_.get_robot_posture(team == T_RED, id);

      const auto x = std::get<0>(pos);
      const auto y = std::get<1>(pos);

      const auto dist_sq = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y);

      // a robot is within the region
      if(dist_sq < target_r * target_r)
        return true;
    }
  }

  return false;
}

void game::publish_current_frame(std::size_t reset_reason)
{
  // get ball and robots position
  const auto g_ball = sv_.get_ball_position();
  std::array<std::array<std::tuple<double, double, double, bool, bool>, c::NUMBER_OF_ROBOTS>, 2> g_robots;
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      const auto r = sv_.get_robot_posture(team == T_RED, id);
      std::get<0>(g_robots[team][id]) = std::get<0>(r);
      std::get<1>(g_robots[team][id]) = std::get<1>(r);
      std::get<2>(g_robots[team][id]) = std::get<2>(r);
      std::get<3>(g_robots[team][id]) = activeness_[team][id];
      std::get<4>(g_robots[team][id]) = touch_[team][id];
    }
  }

  std::vector<std::tuple<std::string, msgpack::object, msgpack::zone> > events;

  {
    for(auto& kv : player_team_infos_) {
      const auto& topic = kv.first;
      auto& ti    = kv.second;

      auto ball = g_ball;
      auto robots = g_robots;
      auto score = score_;
      auto reason = reset_reason;
      std::size_t game_state = game_state_;

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

      // The first frame contains time, score, reset_reason, game_state, and ball ownership
      msg.emplace_back("time", msgpack::object(time_ms_ / 1000., z));
      msg.emplace_back("score", msgpack::object(score, z));
      msg.emplace_back("reset_reason", msgpack::object(reason, z));
      msg.emplace_back("game_state", msgpack::object(game_state, z));
      if(game_state != c::STATE_DEFAULT)
        msg.emplace_back("ball_ownership", msgpack::object(ti.is_red ? ball_ownership_ == T_RED : ball_ownership_ == T_BLUE, z));
      else
        msg.emplace_back("ball_ownership", msgpack::object(false, z));
      msg.emplace_back("half_passed", msgpack::object(half_passed_, z));


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
      msg.emplace_back("coordinates",
                       msgpack::object(std::make_tuple(robots[T_RED],
                                                       robots[T_BLUE],
                                                       ball), z));
      msg.emplace_back("EOF",          msgpack::object(true, z));

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
  time_ms_ = 0;
  score_ = {0, 0};
  half_passed_ = false;

  for(auto& team_activeness : activeness_) {
    for(auto& robot_activeness : team_activeness) {
      robot_activeness = true;
    }
  }

  for(auto& team_touch : touch_) {
    for(auto& robot_touch : team_touch) {
      robot_touch = false;
    }
  }
  recent_touch_ = touch_;

  update_label();

  // reset and wait 1s for stabilizing
  pause();

  // game starts with a kickoff by T_RED
  ball_ownership_ = T_RED;
  game_state_ = c::STATE_KICKOFF;
  kickoff_time_ = time_ms_;

  reset(c::FORMATION_KICKOFF, c::FORMATION_DEFAULT);

  lock_all_robots();
  unlock_robot(ball_ownership_, c::NUMBER_OF_ROBOTS - 1);

  step(c::WAIT_STABLE_MS);

  resume();
  publish_current_frame(c::GAME_START);
  update_label();

  auto reset_reason = c::NONE;

  for(;;) {
    step(c::PERIOD_MS);
    update_label();

    // special case: game ended. finish the game without checking game rules.
    if(time_ms_ >= game_time_ms_) {
      if(half_passed_) {
        if(repeat)
          publish_current_frame(c::EPISODE_END);
        else
          publish_current_frame(c::GAME_END);
        pause();
        stop_robots();
        step(c::WAIT_END_MS);
        return;
      }
      else {
        publish_current_frame(c::HALFTIME);
        pause();
        stop_robots();
        step(c::WAIT_END_MS);

        // mark the halftime and notify the supervisor too
        half_passed_ = true;
        sv_.mark_half_passed();
        time_ms_ = 0;

        // second half starts with a kickoff by T_BLUE
        ball_ownership_ = T_BLUE;
        game_state_ = c::STATE_KICKOFF;
        kickoff_time_ = time_ms_;

        reset(c::FORMATION_DEFAULT, c::FORMATION_KICKOFF);

        lock_all_robots();
        unlock_robot(ball_ownership_, c::NUMBER_OF_ROBOTS - 1);

        step(c::WAIT_STABLE_MS);

        resume();
        publish_current_frame(c::GAME_START);
        update_label();

        reset_reason = c::NONE;
      }
    }

    // publish current frame
    publish_current_frame(reset_reason);

    // update the touch status
    touch_ = sv_.get_robot_touch_ball();
    for(auto& team_touch : touch_) {
      for(auto& robot_touch : team_touch) {
        // if any of the robots has touched the ball at this frame, update recent_touch_
        if(robot_touch) {
          recent_touch_ = touch_;
        }
      }
    }

    reset_reason = c::NONE;

    // check if any of robots has fallen
    {
      for(const auto& team : {T_RED, T_BLUE}) {
        for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
          // if a robot has fallen and could not recover for c::FALL_TIME_MS, send the robot to foulzone
          if(activeness_[team][id] && time_ms_ - fall_time_[team][id] >= c::FALL_TIME_MS) {
            activeness_[team][id] = false;
            sv_.send_to_foulzone(team == T_RED, id);
            sentout_time_[team][id] = time_ms_;
          }
          else {
            // if robot is standing properly
            if (std::get<3>(sv_.get_robot_posture(team == T_RED, id)))
              fall_time_[team][id] = time_ms_;
          }
        }
      }
    }

    // check if any of robots has been left the field without send_to_foulzone()
    {
      for(const auto& team : {T_RED, T_BLUE}) {
        for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
          // an active robot is not in the field
          if(activeness_[team][id] && !robot_in_field(team == T_RED, id)) {
            // make the robot inactive and send out
            activeness_[team][id] = false;
            sv_.send_to_foulzone(team == T_RED, id);
            sentout_time_[team][id] = time_ms_;
          }
        }
      }
    }

    // check if any of robots are in the opponent's goal area
    {
      constexpr auto is_in_opponent_goal = [](double x, double y) {
        return (x > c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH / 2);
      };
      constexpr auto is_in_opponent_goal_area = [](double x, double y) {
        return
        (x <= c::FIELD_LENGTH / 2)
        && (x > c::FIELD_LENGTH / 2 - c::GOAL_AREA_DEPTH)
        && (std::abs(y) < c::GOAL_AREA_WIDTH / 2);
      };

      for(const auto& team : {T_RED, T_BLUE}) {
        for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
          const auto pos = sv_.get_robot_posture(team == T_RED, id);
          const double sign = team == T_RED ? 1 : -1;

          const auto x = sign * std::get<0>(pos);
          const auto y = sign * std::get<1>(pos);

          // if a robot has been in the opponent's goal area for more than c::IOPA_TIME_LIMIT_MS seconds, the robot is relocated to the initial position
          if(is_in_opponent_goal(x, y) || is_in_opponent_goal_area(x,y)) {
            if (time_ms_ - niopa_time_[team][id] >= c::IOPA_TIME_LIMIT_MS) {
              const auto ix = sign * c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][0];
              const auto iy = sign * c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][1];
              const auto r = 1.5 * c::ROBOT_SIZE[id];
              // if any object is located within 1.5 * robot_size, the relocation is delayed
              if(!any_object_nearby(ix, iy, r)) {
                sv_.return_to_field(team == T_RED, id);
                niopa_time_[team][id] = time_ms_;

                // also, as a penalty, the goalkeeper is sent out from the game for c::SENTOUT_DURATION_MS
                activeness_[team][0] = false;
                sv_.send_to_foulzone(team == T_RED, 0);
                sentout_time_[team][0] = time_ms_;
              }
            }
          }
          else {
            niopa_time_[team][id] = time_ms_;
          }
        }
      }
    }

    // check if the goalkeeper is in the own penalty area
    {
      constexpr auto is_in_goal = [](double x, double y) {
        return (x < -c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH / 2);
      };
      constexpr auto is_in_penalty_area = [](double x, double y) {
        return
        (x >= -c::FIELD_LENGTH / 2)
        && (x < -c::FIELD_LENGTH / 2 + c::PENALTY_AREA_DEPTH)
        && (std::abs(y) < c::PENALTY_AREA_WIDTH / 2);
      };

      for(const auto& team : {T_RED, T_BLUE}) {
        // if the goalkeeper is currently not active, skip the check
        if(activeness_[team][0] == false)
          continue;
        const auto pos = sv_.get_robot_posture(team == T_RED, 0);
        const double sign = team == T_RED ? 1 : -1;

        const auto x = sign * std::get<0>(pos);
        const auto y = sign * std::get<1>(pos);

        if(is_in_goal(x, y) || is_in_penalty_area(x,y)) {
          gk_ipa_time_[team] = time_ms_;
        }
        // if the goalkeeper has been not in the penalty area for more than c::GK_NIPA_TIME_LIMIT_MS seconds, the robot is returned to the initial position
        else {
          if (time_ms_ - gk_ipa_time_[team]>= c::GK_NIPA_TIME_LIMIT_MS) {
            const auto ix = sign * c::ROBOT_FORMATION[c::FORMATION_DEFAULT][0][0];
            const auto iy = sign * c::ROBOT_FORMATION[c::FORMATION_DEFAULT][0][1];
            const auto r = 1.5 * c::ROBOT_SIZE[0];
            // if any object is located within 1.5 * robot_size, the return is delayed
            if(!any_object_nearby(ix, iy, r)) {
              sv_.return_to_field(team == T_RED, 0);
              gk_ipa_time_[team] = time_ms_;
            }
          }
        }
      }
    }

    // check rules based on game states
    switch(game_state_) {
    case c::STATE_DEFAULT:
      {
        const auto ball_x = std::get<0>(sv_.get_ball_position());
        const auto ball_y = std::get<1>(sv_.get_ball_position());

        if((std::abs(ball_x) > c::FIELD_LENGTH / 2) && (std::abs(ball_y) < c::GOAL_WIDTH /2)) {
            ++score_[(ball_x > 0) ? T_RED : T_BLUE];
            update_label();

            // stop all and wait for c::WAIT_GOAL seconds
            pause();
            stop_robots();
            step(c::WAIT_GOAL_MS);

            game_state_ = c::STATE_KICKOFF;
            // kickoff_foul_flag_ = false;

            ball_ownership_ = (ball_x > 0) ? T_BLUE : T_RED;
            kickoff_time_ = time_ms_;

            // reset and wait until stabilized
            reset(((ball_ownership_ == T_RED) ? c::FORMATION_KICKOFF : c::FORMATION_DEFAULT), ((ball_ownership_ == T_BLUE) ? c::FORMATION_KICKOFF : c::FORMATION_DEFAULT));

            lock_all_robots();
            unlock_robot(ball_ownership_, c::NUMBER_OF_ROBOTS - 1);

            step(c::WAIT_STABLE_MS);
            resume();

            reset_reason = (ball_x > 0) ? c::SCORE_RED_TEAM : c::SCORE_BLUE_TEAM;
        }
        // ball sent out of the field - proceed to corner kick or goal kick
        else if(!ball_in_field()){
          pause();
          stop_robots();
          step(c::WAIT_STABLE_MS);

          // determine the ownership based on who touched the ball last
          int touch_count[2] = {0, 0};
          for(const auto& team : {T_RED, T_BLUE}) {
            for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
              if(recent_touch_[team][id])
                touch_count[team] += 1;
            }
          }

          // if recent_touch_ was red team dominant, blue team gets the ball
          if(touch_count[T_RED] > touch_count[T_BLUE])
            ball_ownership_ = T_BLUE;
          // the other way around
          else if(touch_count[T_BLUE] > touch_count[T_RED])
            ball_ownership_ = T_RED;
          // otherwise, the attacking team gets an advantage
          else
            ball_ownership_ = (ball_x < 0) ? T_BLUE : T_RED;

          // happened on the left side
          if(ball_x < 0) {
            // if the red gets the ball, proceed to goalkick
            if(ball_ownership_ == T_RED) {
              game_state_ = c::STATE_GOALKICK;
              goalkick_time_ = time_ms_;

              reset(c::FORMATION_GOALKICK_A, c::FORMATION_GOALKICK_D);

              lock_all_robots();
              unlock_robot(ball_ownership_, 0);

              reset_reason = c::GOALKICK;
            }
            // otherwise, proceed to corner kick
            else {
              game_state_ = c::STATE_CORNERKICK;
              cornerkick_time_ = time_ms_;

              if(ball_y > 0) {
                // upper left corner
                reset(c::FORMATION_CAD_AD, c::FORMATION_CAD_AA);
              }
              else {
                // lower left corner
                reset(c::FORMATION_CBC_AD, c::FORMATION_CBC_AA);
              }

              lock_all_robots();
              unlock_robot(ball_ownership_, 4);

              reset_reason = c::CORNERKICK;
            }
          }
          // cornerkick happened on the right side
          else {
            // if the blue gets the ball, proceed to goalkick
            if(ball_ownership_ == T_BLUE) {
              game_state_ = c::STATE_GOALKICK;
              goalkick_time_ = time_ms_;

              reset(c::FORMATION_GOALKICK_D, c::FORMATION_GOALKICK_A);

              lock_all_robots();
              unlock_robot(ball_ownership_, 0);

              reset_reason = c::GOALKICK;
            }
            // otherwise, proceed to corenerkick
            else {
              game_state_ = c::STATE_CORNERKICK;
              cornerkick_time_ = time_ms_;

              if(ball_y > 0) {
                // upper right corner
                reset(c::FORMATION_CBC_AA, c::FORMATION_CBC_AD);
              }
              else {
                // lower right corner
                reset(c::FORMATION_CAD_AA, c::FORMATION_CAD_AD);
              }

              lock_all_robots();
              unlock_robot(ball_ownership_, 4);

              reset_reason = c::CORNERKICK;
            }
          }

          step(c::WAIT_STABLE_MS);
          resume();
          break;
        }
      }

      // check if any of robots should return to the field
      {
        for(const auto& team : {T_RED, T_BLUE}) {
          for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
            // sentout time of 0 is an indicator that the robot is currently on the field
            if(sentout_time_[team][id] == 0)
              continue;
            // if a robot has been sent out and c::SENTOUT_DURATION_MS has passed, return the robot back to the field
            if(time_ms_ - sentout_time_[team][id] >= c::SENTOUT_DURATION_MS) {
              // if any object is located within 1.5 * robot_size, the return is delayed
              const auto s = (team == T_RED) ? 1 : -1;
              const auto x = c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][0] * s;
              const auto y = c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][1] * s;
              const auto r = 1.5 * c::ROBOT_SIZE[id];

              if(!any_object_nearby(x, y, r)) {
                activeness_[team][id] = true;
                sv_.return_to_field(team == T_RED, id);
                sentout_time_[team][id] = 0;
              }
            }
          }
        }
      }

      {
        const auto ball_x = std::get<0>(sv_.get_ball_position());
        // if the penalty area reset condition is met
        if(check_penalty_area()) {
          // the ball ownership is already set by check_penalty_area()
          pause();
          stop_robots();
          step(c::WAIT_STABLE_MS);


          if(ball_x < 0 && ball_ownership_ == T_RED) {
            // proceed to goal kick by Team Red
            game_state_ = c::STATE_GOALKICK;
            reset_reason = c::GOALKICK;
            goalkick_time_ = time_ms_;

            reset(c::FORMATION_GOALKICK_A, c::FORMATION_GOALKICK_D);

            lock_all_robots();
            unlock_robot(ball_ownership_, 0);
          }
          else if(ball_x > 0 && ball_ownership_ == T_BLUE) {
            // proceed to goal kick by Team Blue
            game_state_ = c::STATE_GOALKICK;
            reset_reason = c::GOALKICK;
            goalkick_time_ = time_ms_;

            reset(c::FORMATION_GOALKICK_D, c::FORMATION_GOALKICK_A);

            lock_all_robots();
            unlock_robot(ball_ownership_, 0);
          }
          else if(ball_x < 0 && ball_ownership_ == T_BLUE) {
            // proceed to penalty kick by Team Blue
            game_state_ = c::STATE_PENALTYKICK;
            reset_reason = c::PENALTYKICK;
            penaltykick_time_ = time_ms_;

            reset(c::FORMATION_PENALTYKICK_D, c::FORMATION_PENALTYKICK_A);

            lock_all_robots();
            unlock_robot(ball_ownership_, 4);
          }
          else {
            // proceed to penalty kick by Team Red
            game_state_ = c::STATE_PENALTYKICK;
            reset_reason = c::PENALTYKICK;
            penaltykick_time_ = time_ms_;

            reset(c::FORMATION_PENALTYKICK_A, c::FORMATION_PENALTYKICK_D);

            lock_all_robots();
            unlock_robot(ball_ownership_, 4);
          }

          step(c::WAIT_STABLE_MS);
          resume();
          break;
        }
      }

      if(reset_reason == c::NONE && deadlock_flag_ == true) {
        if(sv_.get_ball_velocity() >= c::DEADLOCK_THRESHOLD) {
          deadlock_time_ = time_ms_;
        }
        // if the ball is not moved fast enough for c::DEADLOCK_DURATION_MS
        else if((time_ms_ - deadlock_time_) >= c::DEADLOCK_DURATION_MS) {
          const auto ball_x = std::get<0>(sv_.get_ball_position());
          const auto ball_y = std::get<1>(sv_.get_ball_position());

          // if the deadlock happened in special region
          if (std::abs(ball_x) > c::FIELD_LENGTH / 2 - c::PENALTY_AREA_DEPTH) {
            // if the deadlock happened inside the penalty area
            if (std::abs(ball_y) < c::PENALTY_AREA_WIDTH / 2) {
              ball_ownership_ = get_pa_ownership();

              pause();
              stop_robots();
              step(c::WAIT_STABLE_MS);


              if(ball_x < 0 && ball_ownership_ == T_RED) {
                // proceed to goal kick by Team Red
                game_state_ = c::STATE_GOALKICK;
                reset_reason = c::GOALKICK;
                goalkick_time_ = time_ms_;

                reset(c::FORMATION_GOALKICK_A, c::FORMATION_GOALKICK_D);

                lock_all_robots();
                unlock_robot(ball_ownership_, 0);
              }
              else if(ball_x > 0 && ball_ownership_ == T_BLUE) {
                // proceed to goal kick by Team Blue
                game_state_ = c::STATE_GOALKICK;
                reset_reason = c::GOALKICK;
                goalkick_time_ = time_ms_;

                reset(c::FORMATION_GOALKICK_D, c::FORMATION_GOALKICK_A);

                lock_all_robots();
                unlock_robot(ball_ownership_, 0);
              }
              else if(ball_x < 0 && ball_ownership_ == T_BLUE) {
                // proceed to penalty kick by Team Blue
                game_state_ = c::STATE_PENALTYKICK;
                reset_reason = c::PENALTYKICK;
                penaltykick_time_ = time_ms_;

                reset(c::FORMATION_PENALTYKICK_D, c::FORMATION_PENALTYKICK_A);

                lock_all_robots();
                unlock_robot(ball_ownership_, 4);
              }
              else {
                // proceed to penalty kick by Team Red
                game_state_ = c::STATE_PENALTYKICK;
                reset_reason = c::PENALTYKICK;
                penaltykick_time_ = time_ms_;

                reset(c::FORMATION_PENALTYKICK_A, c::FORMATION_PENALTYKICK_D);

                lock_all_robots();
                unlock_robot(ball_ownership_, 4);
              }

              step(c::WAIT_STABLE_MS);
              resume();
              deadlock_time_ = time_ms_;
            }
            // if the deadlock happened in the corner regions
            else {
              // set the ball ownership
              ball_ownership_ = get_corner_ownership();

              pause();
              stop_robots();
              step(c::WAIT_STABLE_MS);

              game_state_ = c::STATE_CORNERKICK;

              cornerkick_time_ = time_ms_;

              // determine where to place the robots and the ball
              if (ball_x < 0) { // on Team Red's side
                if (ball_y > 0) { // on upper side
                  if (ball_ownership_ == T_RED) { // ball owned by Team Red
                    reset(c::FORMATION_CAD_DA, c::FORMATION_CAD_DD);
                  }
                  else { // ball owned by Team Blue
                    reset(c::FORMATION_CAD_AD, c::FORMATION_CAD_AA);
                  }
                }
                else { // on lower side
                  if (ball_ownership_ == T_RED) { // ball owned by Team Red
                    reset(c::FORMATION_CBC_DA, c::FORMATION_CBC_DD);
                  }
                  else { // ball owned by Team Blue
                    reset(c::FORMATION_CBC_AD, c::FORMATION_CBC_AA);
                  }
                }
              }
              else { // on Team Blue's side
                if (ball_y > 0) { // on upper side
                  if (ball_ownership_ == T_RED) { // ball owned by Team Red
                    reset(c::FORMATION_CBC_AA, c::FORMATION_CBC_AD);
                  }
                  else { // ball owned by Team Blue
                    reset(c::FORMATION_CBC_DD, c::FORMATION_CBC_DA);
                  }
                }
                else { // on lower side
                  if (ball_ownership_ == T_RED) { // ball owned by Team Red
                    reset(c::FORMATION_CAD_AA, c::FORMATION_CAD_AD);
                  }
                  else { // ball owned by Team Blue
                    reset(c::FORMATION_CAD_DD, c::FORMATION_CAD_DA);
                  }
                }
              }

              lock_all_robots();
              unlock_robot(ball_ownership_, 4);

              step(c::WAIT_STABLE_MS);
              resume();

              reset_reason = c::CORNERKICK;
              deadlock_time_ = time_ms_;
            }
          }
          // if the deadlock happened in the general region, relocate the ball and continue the game
          else {
            pause();
            stop_robots();
            step(c::WAIT_STABLE_MS);

            // determine where to relocate and relocate the ball
            if (ball_x < 0) { // Team Red's region
              if (ball_y > 0) { // upper half
                sv_.relocate_ball(c::BALL_RELOCATION_A);
              }
              else { // lower half
                sv_.relocate_ball(c::BALL_RELOCATION_B);
              }
            }
            else { // Team Blue's region
              if (ball_y > 0) { // upper half
                sv_.relocate_ball(c::BALL_RELOCATION_C);
              }
              else { // lower half
                sv_.relocate_ball(c::BALL_RELOCATION_D);
              }
            }

            sv_.flush_touch_ball();

            step(c::WAIT_STABLE_MS);
            resume();

            reset_reason = c::DEADLOCK;
            deadlock_time_ = time_ms_;
          }
        }
      }
      break;
    case c::STATE_KICKOFF:
      {
        // time limit has passed
        if (time_ms_ - kickoff_time_ >= c::KICKOFF_TIME_LIMIT_MS) {
          game_state_ = c::STATE_DEFAULT;
          unlock_all_robots();
        }
        else {
          const auto ball_x = std::get<0>(sv_.get_ball_position());
          const auto ball_y = std::get<1>(sv_.get_ball_position());
          // ball has left the center circle
          if (ball_x*ball_x + ball_y*ball_y > c::KICKOFF_BORDER*c::KICKOFF_BORDER) {
            unlock_all_robots();
            game_state_ = c::STATE_DEFAULT;
          }
        }

        deadlock_time_ = time_ms_;
      }
      break;
    case c::STATE_GOALKICK:
      {
        // time limit has passed
        if (time_ms_ - goalkick_time_ >= c::GOALKICK_TIME_LIMIT_MS) {
          game_state_ = c::STATE_DEFAULT;
          unlock_all_robots();
        }
        // the goalie has touched the ball
        else if (touch_[ball_ownership_][0]) {
          game_state_ = c::STATE_DEFAULT;
          unlock_all_robots();
        }
        deadlock_time_ = time_ms_;
      }
      break;
    case c::STATE_CORNERKICK:
      {
        // time limit has passed
        if (time_ms_ - cornerkick_time_ >= c::CORNERKICK_TIME_LIMIT_MS) {
          game_state_ = c::STATE_DEFAULT;
          unlock_all_robots();
        }
        else {
          // a robot has touched the ball
          for (std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
            if (touch_[ball_ownership_][id]) {
              game_state_ = c::STATE_DEFAULT;
              unlock_all_robots();
              break;
            }
          }
        }
        deadlock_time_ = time_ms_;
      }
      break;
    case c::STATE_PENALTYKICK:
      {
        // time limit has passed
        if (time_ms_ - penaltykick_time_ >= c::PENALTYKICK_TIME_LIMIT_MS) {
          game_state_ = c::STATE_DEFAULT;
          unlock_all_robots();
        }
        // the attacker has touched the ball
        else if (touch_[ball_ownership_][4]) {
          game_state_ = c::STATE_DEFAULT;
          unlock_all_robots();
        }
        deadlock_time_ = time_ms_;
      }
      break;
    default:
      break;
    }
  }
}

// called from io thread
void game::on_bootup(autobahn::wamp_invocation invocation)
{
  const std::string key = invocation->argument<std::string>(0);

  const auto it = player_team_infos_.find(key);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  if(state_.load() != STATE_WAITING_BOOTUP) {
    invocation->empty_result();
    return;
  }

  auto& ti = it->second;
  ti.is_bootup = true;

  bootup_waiting_list_.push_back(invocation);

  if(std::all_of(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                 [](const auto& kv) { return kv.second.is_bootup == true; })) {
    // std::cout << "Everyone bootup" << std::endl;
    state_.store(STATE_WAITING_READY);
    for(auto& inv : bootup_waiting_list_) {
      inv->empty_result();
    }
    bootup_waiting_list_.clear();
    bootup_promise_.set_value();
  }
}

// called from io thread
void game::on_ready(autobahn::wamp_invocation invocation)
{
  const std::string key = invocation->argument<std::string>(0);

  assert(state_.load() != STATE_WAITING_BOOTUP);

  const auto it = player_team_infos_.find(key);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  auto& ti = it->second;

  { // reset image buffer
    std::unique_lock<std::mutex> lck(ti.m);
    ti.imbuf.reset();
  }

  if(state_.load() != STATE_STARTED) {
    ti.is_ready = true;

    if(std::all_of(std::cbegin(player_team_infos_), std::cend(player_team_infos_),
                   [](const auto& kv) { return kv.second.is_ready == true; })) {
      ready_promise_.set_value();
    }
  }

  invocation->empty_result();
}

// called from io thread
void game::on_info(autobahn::wamp_invocation invocation)
{
  const std::string caller = invocation->argument<std::string>(0);

  // if the caller is not a player, then error
  auto it = player_team_infos_.find(caller);
  if(it == std::cend(player_team_infos_)) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }
  auto& ti = it->second;

  invocation->result(std::make_tuple(info_[ti.is_red ? T_RED : T_BLUE]));
}

// called from io thread. it just sets the wheel speed and not transfer it to the simulator.
void game::on_set_speed(autobahn::wamp_invocation invocation)
{
  const auto caller      = invocation->argument<std::string>(0);

  // if the caller is not a player, then error
  auto it = player_team_infos_.find(caller);
  if((it == std::cend(player_team_infos_))
     || (it->second.role != ROLE_PLAYER)
     ) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  if(!paused_.load()) { // if it's paused, ignore the call and return
    const bool is_red = it->second.is_red;

    const auto ws = invocation->argument<std::array<double, 10> >(1);

    for(std::size_t i = 0; i < c::NUMBER_OF_ROBOTS; ++i) {
      io_thread_wheel_speed_[is_red ? T_RED : T_BLUE][i][0] = ws[2*i + 0];
      io_thread_wheel_speed_[is_red ? T_RED : T_BLUE][i][1] = ws[2*i + 1];
    }

    wheel_speed_.write(io_thread_wheel_speed_);
  }

  invocation->empty_result();
}

// called from io thread.
void game::on_commentate(autobahn::wamp_invocation invocation)
{
  const auto caller      = invocation->argument<std::string>(0);

  // if the caller is not a commentator, then error
  auto it = player_team_infos_.find(caller);
  if((it == std::cend(player_team_infos_))
     || (it->second.role != ROLE_COMMENTATOR)
     ) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  std::unique_lock<std::mutex> lck(m_comments_);
  if(half_passed_ == false)
    comments_.push_back((boost::format("[%.2f] ") % (time_ms_ / 1000.)).str() + invocation->argument<std::string>(1));
  else
    comments_.push_back((boost::format("[%.2f] ") % ((game_time_ms_ + time_ms_) / 1000.)).str() + invocation->argument<std::string>(1));

  invocation->empty_result();
}

// called from io thread.
void game::on_report(autobahn::wamp_invocation invocation)
{
  const auto caller      = invocation->argument<std::string>(0);

  // if the caller is not a reporter, then error
  auto it = player_team_infos_.find(caller);
  if((it == std::cend(player_team_infos_))
     || (it->second.role != ROLE_REPORTER)
     ) {
    invocation->error("wamp.error.invalid_argument");
    return;
  }

  report = invocation->argument<std::vector<std::string>>(1);

  // we will handle this report internally.

  invocation->empty_result();
}
