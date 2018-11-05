// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "game.hpp"

#include <boost/format.hpp>
#include <boost/random/random_device.hpp>

#include <random>
#include <string>

#include "rapidjson/document.h"
#include <fstream>

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
{
  for(auto& fc : foul_pa_counter_) {
    fc.set_capacity(c::FOUL_PA_DURATION_MS / c::PERIOD_MS);
  }

  for(auto& fc : foul_opa_counter_) {
    fc.set_capacity(c::FOUL_PA_DURATION_MS / c::PERIOD_MS);
  }

  for(auto& fc : foul_ga_counter_) {
    fc.set_capacity(c::FOUL_GA_DURATION_MS / c::PERIOD_MS);
  }

  for(auto& fc : foul_oga_counter_) {
    fc.set_capacity(c::FOUL_GA_DURATION_MS / c::PERIOD_MS);
  }
}

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

  //gets game rules from 'config.json' (if no rules specified, default options are given)
  {
    game_time_ms_ = c::DEFAULT_GAME_TIME_MS / c::PERIOD_MS * c::PERIOD_MS;
    deadlock_flag_ = true;
    goal_area_foul_flag_ = true;
    penalty_area_foul_flag_ = true;

    for (int i = 0; i < 2; i++)
      for (unsigned int j = 0; j < c::NUMBER_OF_ROBOTS; j++) {
        // stop_time_[i][j] = 0;
        exhausted_[i][j] = false;
    }

    if (config_json.HasMember("rule") && config_json["rule"].IsObject()) { //set rules
      if (config_json["rule"].HasMember("game_time") && config_json["rule"]["game_time"].IsNumber())
        game_time_ms_ = static_cast<size_t>(config_json["rule"]["game_time"].GetDouble() * 1000) / c::PERIOD_MS * c::PERIOD_MS;

      if (config_json["rule"].HasMember("deadlock") && config_json["rule"]["deadlock"].IsBool())
        deadlock_flag_ = config_json["rule"]["deadlock"].GetBool();

      if (config_json["rule"].HasMember("goal_area_foul") && config_json["rule"]["goal_area_foul"].IsBool())
        goal_area_foul_flag_ = config_json["rule"]["goal_area_foul"].GetBool();

      if (config_json["rule"].HasMember("penalty_area_foul") && config_json["rule"]["penalty_area_foul"].IsBool())
        penalty_area_foul_flag_ = config_json["rule"]["penalty_area_foul"].GetBool();
    }
    else
      std::cout << "\"rule\" section of 'config.json' seems to be missing: using default options" << std::endl;

    std::cout << "Rules:" << std::endl;
    std::cout << "     game duration - " << game_time_ms_ / 1000.0 << " seconds" << std::endl;
    std::cout << "          deadlock - " << (deadlock_flag_ ? "on" : "off") << std::endl;
    std::cout << "    goal area foul - " << (goal_area_foul_flag_ ? "on" : "off") << std::endl;
    std::cout << " penalty area foul - " << (penalty_area_foul_flag_ ? "on" : "off") << std::endl;
  }

  const auto path_prefix = std::string("../../");

  // gets the teams' information from 'config.json'
  assert(config_json.HasMember("team_a") && config_json["team_a"].IsObject());
  assert(config_json.HasMember("team_b") && config_json["team_b"].IsObject());
  for(const auto& team : {T_RED, T_BLUE}) {
    const auto tc = ((team == T_RED) ? "team_a" : "team_b");
    const auto tc_op = ((team != T_RED) ? "team_a" : "team_b");

    // my team
    const std::string& name   = ((config_json[tc].HasMember("name") && config_json[tc]["name"].IsString()) ? config_json[tc]["name"].GetString() : "");
    const double&      rating = 0; //rating is currently disabled
    const std::string& exe    = ((config_json[tc].HasMember("executable") && config_json[tc]["executable"].IsString()) ? config_json[tc]["executable"].GetString() : "");
    const std::string& data   = ((config_json[tc].HasMember("datapath") && config_json[tc]["datapath"].IsString()) ? config_json[tc]["datapath"].GetString() : "");

    // opponent
    const std::string& name_op   = ((config_json[tc_op].HasMember("name") && config_json[tc_op]["name"].IsString()) ? config_json[tc_op]["name"].GetString() : "");
    const double&      rating_op = 0; //rating is currently disabled

    const auto ret = player_team_infos_.emplace(std::piecewise_construct,
                                                std::make_tuple(random_string(c::KEY_LENGTH)),
                                                std::make_tuple(name, rating, path_prefix + exe, path_prefix + data,
                                                                ROLE_PLAYER, team == T_RED)
                                                );

    assert(ret.second);

    std::cout << ((team == T_RED) ? "Team A: " : "Team B: ") << std::endl;
    std::cout << "  team name - " << name << std::endl;
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
    info.emplace_back("max_meters_run", msgpack::object(c::MAX_METERS_RUN, z_info_));

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
                                                  std::make_tuple(name, 0, path_prefix + exe, data,
                                                                  ROLE_COMMENTATOR, true)
                                                  );

      assert(ret.second);

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
                                                  std::make_tuple(name, 0, path_prefix + exe, data,
                                                                  ROLE_REPORTER, true)
                                                  );

      assert(ret.second);

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

  // bootup VMs & wait until app players boot up
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

      // TODO: send winning frame to team pointed by it.
    }
    else {
      try {
        std::cout << "Starting a new game" << std::endl;
        run_game();

        // now players have c::WAIT_KILL seconds to finish
        const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(c::WAIT_KILL_MS);

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
      ti.c = bp::child(bp::exe = ti.executable,
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
                         bp::args = {ti.executable,
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
  sv_.setLabel(0,
               (boost::format("score %d:%d, time %.2f") % score_[0] % score_[1] % (time_ms_ / 1000.)).str(),
               0.4, 0.95, // x, y
               0.10, 0x00000000, // size, color
               0, "Arial" // transparency, font
               );

  constexpr std::size_t comments_start = 1;

  std::unique_lock<std::mutex> lck(m_comments_);
  for(std::size_t i = 0; i < comments_.size(); ++i) {
    sv_.setLabel(comments_start + i,
                 comments_[i],
                 0.01, 0.02 + 0.04 * i, // x, y
                 0.08, 0x00000000, // size, color
                 0, "Arial" // transparency, font
                 );
  }
}

void game::save_current_pos()
{
  for(const auto& team : {T_RED, T_BLUE}) {
    auto is_red = team == T_RED;
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      const auto pos = sv_.get_robot_posture(is_red, id);

      prev_pos_[team][id] = pos;
    }
  }
}

void game::update_meters_run()
{
  for(const auto& team : {T_RED, T_BLUE}) {
    auto is_red = team == T_RED;
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      const auto pos = sv_.get_robot_posture(is_red, id);

      const auto x = std::get<0>(pos);
      const auto y = std::get<1>(pos);

      const auto stand = std::get<3>(pos);

      const auto prev_x = std::get<0>(prev_pos_[team][id]);
      const auto prev_y = std::get<1>(prev_pos_[team][id]);

      if(activeness_[team][id] && stand) {
        meters_run_[team][id] += sqrt(pow(x - prev_x,2) + pow(y - prev_y,2));
        if(meters_run_[team][id] >= c::MAX_METERS_RUN[id]) {
          meters_run_[team][id] = c::MAX_METERS_RUN[id];
          activeness_[team][id] = false;
          exhausted_[team][id] = true;
        }
      }
    }
  }

  // sv_.setLabel(25,
  //              (boost::format("Max available: %.2f m, %.2f m, %.2f m") % c::MAX_METERS_RUN[0] % c::MAX_METERS_RUN[2] % c::MAX_METERS_RUN[4]).str(),
  //              0, 0, // x, y
  //              0.08, 0x00000000, // size, color
  //              0, "Arial" // transparency, font
  //              );
  //
  // sv_.setLabel(26,
  //              (boost::format("Red 0[%.2f]: %.2f m\nRed 1[%.2f]: %.2f m\nRed 2[%.2f]: %.2f m\nRed 3[%.2f]: %.2f m\nRed 4[%.2f]: %.2f m") % stop_time_[0][0] % meters_run_[0][0] % stop_time_[0][1] % meters_run_[0][1] % stop_time_[0][2] % meters_run_[0][2] % stop_time_[0][3] % meters_run_[0][3] % stop_time_[0][4] % meters_run_[0][4]).str(),
  //              0, 0.04, // x, y
  //              0.08, 0x00000000, // size, color
  //              0, "Arial" // transparency, font
  //              );
  //
  // sv_.setLabel(27,
  //             (boost::format("Blue 0[%.2f]: %.2f m\nBlue 1[%.2f]: %.2f m\nBlue 2[%.2f]: %.2f m\nBlue 3[%.2f]: %.2f m\nBlue 4[%.2f]: %.2f m") % stop_time_[1][0] % meters_run_[1][0] % stop_time_[1][1] % meters_run_[1][1] % stop_time_[1][2] % meters_run_[1][2] % stop_time_[1][3] % meters_run_[1][3] % stop_time_[1][4] % meters_run_[1][4]).str(),
  //             0.77, 0.04, // x, y
  //             0.08, 0x00000000, // size, color
  //             0, "Arial" // transparency, font
  //             );
}

// apply penalty to robots sent out
void game::apply_penalty(bool is_red, std::size_t id)
{
  auto team = (is_red ? T_RED : T_BLUE);
  if (!exhausted_[team][id]) {
    meters_run_[team][id] += c::DEFAULT_PENALTY_RATIO * c::MAX_METERS_RUN[id];

    if (meters_run_[team][id] >= c::MAX_METERS_RUN[id]) {
      meters_run_[team][id] = c::MAX_METERS_RUN[id];
      activeness_[team][id] = false;
      exhausted_[team][id] = true;
      //sv_.send_to_foulzone(is_red, id);
      // if(stop_time_[team][id] == 0)
      //   stop_time_[team][id] = time_ms_ / 1000.;
    }
  }
}

// game state control functions
void game::step(std::size_t ms, bool update)
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

    if(update)
      update_meters_run();
    save_current_pos();

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

  // check stamina status and send exhausted robots out
  for(const auto& team : {T_RED, T_BLUE}) {
    auto is_red = team == T_RED;
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      if(exhausted_[team][id]) {
        activeness_[team][id] = false;
        sv_.send_to_foulzone(is_red, id);
      }
    }
  }

  // reset in_penalty_area
  for(auto& team_ipa : in_penalty_area_) {
    for(auto& robot_ipa : team_ipa) {
      robot_ipa = false;
    }
  }

  // reset in_opponent_penalty_area
  for(auto& team_iopa : in_opponent_penalty_area_) {
    for(auto& robot_iopa : team_iopa) {
      robot_iopa = false;
    }
  }

  // reset in_goal_area
  for(auto& team_iga : in_goal_area_) {
    for(auto& robot_iga : team_iga) {
      robot_iga = false;
    }
  }

  // reset in_opponent_goal_area
  for(auto& team_ioga : in_opponent_goal_area_) {
    for(auto& robot_ioga : team_ioga) {
      robot_ioga = false;
    }
  }

  stop_robots();

  // reset foul counters
  for(auto& fc : foul_pa_counter_) {
    fc.clear();
  }
  for(auto& fc : foul_opa_counter_) {
    fc.clear();
  }
  for(auto& fc : foul_ga_counter_) {
    fc.clear();
  }
  for(auto& fc : foul_oga_counter_) {
    fc.clear();
  }

  deadlock_reset_time_ = time_ms_;
  deadlock_time_ = time_ms_;
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

std::size_t game::count_robots_in_goal_area(bool is_red)
{
  std::size_t ret = 0;

  constexpr auto is_in_goal = [](double x, double y) {
    return (x < -c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH / 2);
  };

  constexpr auto is_in_goal_area = [](double x, double y) {
    return
    (x >= -c::FIELD_LENGTH / 2)
    && (x < -c::FIELD_LENGTH / 2 + c::GOAL_AREA_DEPTH)
    && (std::abs(y) < c::GOAL_AREA_WIDTH / 2);
  };

  for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
    const auto pos = sv_.get_robot_posture(is_red, id);
    const double sign = is_red ? 1 : -1;

    const auto x = sign * std::get<0>(pos);
    const auto y = sign * std::get<1>(pos);

    if(is_in_goal(x, y) || is_in_goal_area(x, y)) {
      in_goal_area_[is_red ? T_RED : T_BLUE][id] = true;
      ++ret;
    }
    else {
      in_goal_area_[is_red ? T_RED : T_BLUE][id] = false;
    }
  }

  return ret;
}

std::size_t game::count_robots_in_opponent_goal_area(bool is_red)
{
  std::size_t ret = 0;

  constexpr auto is_in_opponent_goal = [](double x, double y) {
    return (x > c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH / 2);
  };

  constexpr auto is_in_opponent_goal_area = [](double x, double y) {
    return
    (x <= c::FIELD_LENGTH / 2)
    && (x > c::FIELD_LENGTH / 2 - c::GOAL_AREA_DEPTH)
    && (std::abs(y) < c::GOAL_AREA_WIDTH / 2);
  };

  for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
    const auto pos = sv_.get_robot_posture(is_red, id);
    const double sign = is_red ? 1 : -1;

    const auto x = sign * std::get<0>(pos);
    const auto y = sign * std::get<1>(pos);

    if(is_in_opponent_goal(x, y) || is_in_opponent_goal_area(x, y)) {
      in_opponent_goal_area_[is_red ? T_RED : T_BLUE][id] = true;
      ++ret;
    }
    else {
      in_opponent_goal_area_[is_red ? T_RED : T_BLUE][id] = false;
    }
  }

  return ret;
}

std::size_t game::count_robots_in_penalty_area(bool is_red)
{
  std::size_t ret = 0;

  constexpr auto is_in_goal = [](double x, double y) {
    return (x < -c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH / 2);
  };

  constexpr auto is_in_penalty_area = [](double x, double y) {
    return
    (x >= -c::FIELD_LENGTH / 2)
    && (x < -c::FIELD_LENGTH / 2 + c::PENALTY_AREA_DEPTH)
    && (std::abs(y) < c::PENALTY_AREA_WIDTH / 2);
  };

  for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
    const auto pos = sv_.get_robot_posture(is_red, id);
    const double sign = is_red ? 1 : -1;

    const auto x = sign * std::get<0>(pos);
    const auto y = sign * std::get<1>(pos);

    if(is_in_goal(x, y) || is_in_penalty_area(x, y)) {
      in_penalty_area_[is_red ? T_RED : T_BLUE][id] = true;
      ++ret;
    }
    else {
      in_penalty_area_[is_red ? T_RED : T_BLUE][id] = false;
    }
  }

  return ret;
}

std::size_t game::count_robots_in_opponent_penalty_area(bool is_red)
{
  std::size_t ret = 0;

  constexpr auto is_in_opponent_goal = [](double x, double y) {
    return (x > c::FIELD_LENGTH / 2) && (std::abs(y) < c::GOAL_WIDTH / 2);
  };

  constexpr auto is_in_opponent_penalty_area = [](double x, double y) {
    return
    (x <= c::FIELD_LENGTH / 2)
    && (x > c::FIELD_LENGTH / 2 - c::PENALTY_AREA_DEPTH)
    && (std::abs(y) < c::PENALTY_AREA_WIDTH / 2);
  };

  for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
    const auto pos = sv_.get_robot_posture(is_red, id);
    const double sign = is_red ? 1 : -1;

    const auto x = sign * std::get<0>(pos);
    const auto y = sign * std::get<1>(pos);

    if(is_in_opponent_goal(x, y) || is_in_opponent_penalty_area(x, y)) {
      in_opponent_penalty_area_[is_red ? T_RED : T_BLUE][id] = true;
      ++ret;
    }
    else {
      in_opponent_penalty_area_[is_red ? T_RED : T_BLUE][id] = false;
    }
  }

  return ret;
}

void game::publish_current_frame(std::size_t reset_reason)
{
  // get ball and robots position
  const auto g_ball = sv_.get_ball_position();
  const auto g_touch = sv_.get_robot_touch_ball();
  std::array<std::array<std::tuple<double, double, double, bool, bool, double>, c::NUMBER_OF_ROBOTS>, 2> g_robots;
  for(const auto& team : {T_RED, T_BLUE}) {
    for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
      const auto r = sv_.get_robot_posture(team == T_RED, id);
      std::get<0>(g_robots[team][id]) = std::get<0>(r);
      std::get<1>(g_robots[team][id]) = std::get<1>(r);
      std::get<2>(g_robots[team][id]) = std::get<2>(r);
      std::get<3>(g_robots[team][id]) = activeness_[team][id];
      std::get<4>(g_robots[team][id]) = g_touch[team][id];
      std::get<5>(g_robots[team][id]) = meters_run_[team][id];
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
  time_ms_ = 0;
  score_ = {0, 0};

  for(auto& team_activeness : activeness_) {
    for(auto& robot_activeness : team_activeness) {
      robot_activeness = true;
    }
  }

  for(auto& team_ipa : in_penalty_area_) {
    for(auto& robot_ipa : team_ipa) {
      robot_ipa = false;
    }
  }

  for(auto& team_ipa : in_opponent_penalty_area_) {
    for(auto& robot_ipa : team_ipa) {
      robot_ipa = false;
    }
  }

  for(auto& team_iga : in_goal_area_) {
    for(auto& robot_iga : team_iga) {
      robot_iga = false;
    }
  }

  for(auto& team_iga : in_opponent_goal_area_) {
    for(auto& robot_iga : team_iga) {
      robot_iga = false;
    }
  }

  for(auto& team_mr : meters_run_) {
    for(auto& robot_mr : team_mr) {
      robot_mr = 0.0;
    }
  }

  update_label();

  // reset and wait 1s for stabilizing
  pause();

  // game starts with a backpass by T_RED
  ball_ownership_ = T_RED;
  game_state_ = c::STATE_BACKPASS;
  backpass_time_ = time_ms_;
  backpass_foul_flag_ = false;

  reset(c::FORMATION_BACKPASS, c::FORMATION_DEFAULT);

  lock_all_robots();
  unlock_robot(ball_ownership_, c::NUMBER_OF_ROBOTS - 1);

  step(c::WAIT_STABLE_MS, false);

  resume();
  publish_current_frame(c::GAME_START);
  update_label();

  auto reset_reason = c::NONE;

  for(;;) {
    step(c::PERIOD_MS, true);
    update_label();

    // special case: game ended. finish the game without checking game rules.
    if(time_ms_ >= game_time_ms_) {
      publish_current_frame(c::GAME_END);
      pause();
      stop_robots();
      step(c::WAIT_END_MS, false);
      return;
    }

    // special case 2: all robots are exhausted. finish the game without checking game rules.
    if(std::all_of(std::begin(exhausted_[0]), std::end(exhausted_[0]), [](const auto& is_exhausted) { return is_exhausted; })
     &&std::all_of(std::begin(exhausted_[1]), std::end(exhausted_[1]), [](const auto& is_exhausted) { return is_exhausted; })) {
      publish_current_frame(c::GAME_END);
      pause();
      stop_robots();
      step(c::WAIT_END_MS, false);
      return;
    }

    // publish current frame
    publish_current_frame(reset_reason);
    reset_reason = c::NONE;

    // check rules based on game states
    switch(game_state_) {
    case c::STATE_DEFAULT:
      { // if a team scored
        const auto ball_x = std::get<0>(sv_.get_ball_position());
        const auto ball_y = std::get<1>(sv_.get_ball_position());
        if((std::abs(ball_x) > c::FIELD_LENGTH / 2) && (std::abs(ball_y) < c::GOAL_WIDTH /2)) {
          ++score_[(ball_x > 0) ? T_RED : T_BLUE];
          update_label();

          // stop all and wait for c::WAIT_GOAL seconds
          pause();
          stop_robots();
          step(c::WAIT_GOAL_MS, false);

          game_state_ = c::STATE_BACKPASS;
          backpass_foul_flag_ = false;

          ball_ownership_ = (ball_x > 0) ? T_BLUE : T_RED;
          backpass_time_ = time_ms_;

          // reset and wait until stabilized
          reset(((ball_ownership_ == T_RED) ? c::FORMATION_BACKPASS : c::FORMATION_DEFAULT), ((ball_ownership_ == T_BLUE) ? c::FORMATION_BACKPASS : c::FORMATION_DEFAULT));

          lock_all_robots();
          unlock_robot(ball_ownership_, c::NUMBER_OF_ROBOTS - 1);

          step(c::WAIT_STABLE_MS, false);
          resume();

          reset_reason = (ball_x > 0) ? c::SCORE_RED_TEAM : c::SCORE_BLUE_TEAM;
        }
      }

      // if the ball is not moved fast enough for c::DEADLOCK_RESET_MS
      if(reset_reason == c::NONE && deadlock_flag_ == true) {
        if(sv_.get_ball_velocity() >= c::DEADLOCK_THRESHOLD) {
          deadlock_reset_time_ = time_ms_;
        }
        else if((time_ms_ - deadlock_reset_time_) >= c::DEADLOCK_RESET_MS) {
          pause();
          stop_robots();
          reset(c::FORMATION_DEFAULT, c::FORMATION_DEFAULT);
          step(c::WAIT_STABLE_MS, false);
          resume();

          reset_reason = c::DEADLOCK;
        }
      }

      // if a team is blocking the goal area
      if (goal_area_foul_flag_ == true) {
        for(const auto& team : {T_RED, T_BLUE}) {
          {
            auto cnt_rbts_iga = count_robots_in_goal_area(team == T_RED);
            foul_ga_counter_[team].push_back(cnt_rbts_iga);

            const auto sum = std::accumulate(std::cbegin(foul_ga_counter_[team]), std::cend(foul_ga_counter_[team]), (std::size_t)0);
            if((cnt_rbts_iga >= c::FOUL_GA_THRESHOLD) && (sum >= c::FOUL_GA_THRESHOLD * foul_ga_counter_[team].capacity())) {
              std::mt19937 rng{std::random_device{}()};
              std::uniform_int_distribution<std::size_t> dist(0, 4);

              auto& team_activeness = activeness_[team];

              if(std::any_of(std::begin(team_activeness), std::end(team_activeness),
                             [](const auto& is_active) { return is_active; })) {
                for(;;) {
                  std::size_t id = dist(rng);
                  auto& is_active = activeness_[team][id];
                  auto& is_iga = in_goal_area_[team][id];
                  if(is_active && is_iga) {
                    is_active = false;
                    is_iga = false;
                    apply_penalty(team == T_RED, id);
                    sv_.send_to_foulzone(team == T_RED, id);
                    break;
                  }
                }
              }
              foul_ga_counter_[team].clear();
            }
          }
          {
            auto cnt_rbts_ioga = count_robots_in_opponent_goal_area(team == T_RED);
            foul_oga_counter_[team].push_back(cnt_rbts_ioga);

            const auto sum = std::accumulate(std::cbegin(foul_oga_counter_[team]), std::cend(foul_oga_counter_[team]), (std::size_t)0);
            if((cnt_rbts_ioga >= c::FOUL_GA_THRESHOLD) && (sum >= c::FOUL_GA_THRESHOLD * foul_oga_counter_[team].capacity())) {
              std::mt19937 rng{std::random_device{}()};
              std::uniform_int_distribution<std::size_t> dist(0, 4);

              auto& team_activeness = activeness_[team];

              if(std::any_of(std::begin(team_activeness), std::end(team_activeness),
                             [](const auto& is_active) { return is_active; })) {
                for(;;) {
                  std::size_t id = dist(rng);
                  auto& is_active = activeness_[team][id];
                  auto& is_ioga = in_opponent_goal_area_[team][id];
                  if(is_active && is_ioga) {
                    is_active = false;
                    is_ioga = false;
                    apply_penalty(team == T_RED, id);
                    sv_.send_to_foulzone(team == T_RED, id);
                    break;
                  }
                }
              }
              foul_oga_counter_[team].clear();
            }
          }
        }
      }

      // if a team is blocking the penalty area
      if (penalty_area_foul_flag_ == true) {
        for(const auto& team : {T_RED, T_BLUE}) {
          {
            auto cnt_rbts_ipa = count_robots_in_penalty_area(team == T_RED);
            foul_pa_counter_[team].push_back(cnt_rbts_ipa);

            const auto sum = std::accumulate(std::cbegin(foul_pa_counter_[team]), std::cend(foul_pa_counter_[team]), (std::size_t)0);
            if((cnt_rbts_ipa >= c::FOUL_PA_THRESHOLD) && (sum >= c::FOUL_PA_THRESHOLD * foul_pa_counter_[team].capacity())) {
              std::mt19937 rng{std::random_device{}()};
              std::uniform_int_distribution<std::size_t> dist(0, 4);

              auto& team_activeness = activeness_[team];

              if(std::any_of(std::begin(team_activeness), std::end(team_activeness),
                             [](const auto& is_active) { return is_active; })) {
                for(;;) {
                  std::size_t id = dist(rng);
                  auto& is_active = activeness_[team][id];
                  auto& is_ipa = in_penalty_area_[team][id];
                  if(is_active && is_ipa) {
                    is_active = false;
                    is_ipa = false;
                    apply_penalty(team == T_RED, id);
                    sv_.send_to_foulzone(team == T_RED, id);
                    break;
                  }
                }
              }
              foul_pa_counter_[team].clear();
            }
          }
          {
            auto cnt_rbts_iopa = count_robots_in_opponent_penalty_area(team == T_RED);
            foul_opa_counter_[team].push_back(cnt_rbts_iopa);

            const auto sum = std::accumulate(std::cbegin(foul_opa_counter_[team]), std::cend(foul_opa_counter_[team]), (std::size_t)0);
            if((cnt_rbts_iopa >= c::FOUL_PA_THRESHOLD) && (sum >= c::FOUL_PA_THRESHOLD * foul_opa_counter_[team].capacity())) {
              std::mt19937 rng{std::random_device{}()};
              std::uniform_int_distribution<std::size_t> dist(0, 4);

              auto& team_activeness = activeness_[team];

              if(std::any_of(std::begin(team_activeness), std::end(team_activeness),
                             [](const auto& is_active) { return is_active; })) {
                for(;;) {
                  std::size_t id = dist(rng);
                  auto& is_active = activeness_[team][id];
                  auto& is_iopa = in_opponent_penalty_area_[team][id];
                  if(is_active && is_iopa) {
                    is_active = false;
                    is_iopa = false;
                    apply_penalty(team == T_RED, id);
                    sv_.send_to_foulzone(team == T_RED, id);
                    break;
                  }
                }
              }
              foul_opa_counter_[team].clear();
            }
          }
        }
      }

      // if the ball is not moved fast enough for c::DEADLOCK_DURATION_MS
      if(deadlock_flag_ == true) {
        if(sv_.get_ball_velocity() >= c::DEADLOCK_THRESHOLD) {
          deadlock_time_ = time_ms_;
        }
        else if((time_ms_ - deadlock_time_) >= c::DEADLOCK_DURATION_MS) {
          for(const auto& team : {T_RED, T_BLUE}) {
            for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; id++) {
              if(activeness_[team][id] && (sv_.get_distance_from_ball(team == T_RED, id) < c::DEADLOCK_RANGE)) {
                activeness_[team][id] = false;
                apply_penalty(team == T_RED, id);
                sv_.send_to_foulzone(team == T_RED, id);
              }
            }
          }
          deadlock_time_ = time_ms_;
        }
      }
      break;
    case c::STATE_BACKPASS:
      {
        // time limit has passed
        if (time_ms_ - backpass_time_ >= c::BACKPASS_TIME_LIMIT_MS) {
          game_state_ = c::STATE_DEFAULT;
        }
        // ball has left the center circle
        const auto ball_x = std::get<0>(sv_.get_ball_position());
        const auto ball_y = std::get<1>(sv_.get_ball_position());
        if (ball_x*ball_x + ball_y*ball_y > c::BACKPASS_BORDER*c::BACKPASS_BORDER) {
          // if ((ball_ownership_ == T_RED) ? (ball_x < 0) : (ball_x > 0)) { // good backpass
          backpass_foul_flag_ = false;

          unlock_all_robots();

          game_state_ = c::STATE_DEFAULT;
          // }
          // else { // bad backpass
            // if (backpass_foul_flag_) { // both teams made bad backpasses
              // std::cout << "Two bad backpass in a row!" << std::endl;
              // game_state_ = c::STATE_DEFAULT;

              // pause();
              // stop_robots();
              // reset(c::FORMATION_DEFAULT, c::FORMATION_DEFAULT);

              // step(c::WAIT_STABLE_MS, false);
              // resume();
            // }
            // else { // the team made a bad backpass => ownership is changed
              // std::cout << "Wrong backpass!" << std::endl;
              // ball_ownership_ = !ball_ownership_;
              // backpass_time_ = time_ms_;
              // backpass_foul_flag_ = true;

              // pause();
              // stop_robots();
              // reset(((ball_ownership_ == T_RED) ? c::FORMATION_BACKPASS : c::FORMATION_DEFAULT), ((ball_ownership_ == T_BLUE) ? c::FORMATION_BACKPASS : c::FORMATION_DEFAULT));

              // lock_all_robots();
              // unlock_robot(ball_ownership_, c::NUMBER_OF_ROBOTS - 1);

              // step(c::WAIT_STABLE_MS, false);
              // resume();
            // }
          // }
        }
      }
      break;
    case c::STATE_FREEKICK:
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
    std::cout << "Everyone bootup" << std::endl;
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
  comments_.push_back((boost::format("[%.2f] ") % (time_ms_ / 1000.)).str() + invocation->argument<std::string>(1));

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
