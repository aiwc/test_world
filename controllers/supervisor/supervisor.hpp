// File:              supervisor.hpp
// Date:              Jan. 23, 2018
// Description:       AI World Cup supervisor header
// Author(s):         Inbae Jeong, Chansol Hong
// Current Developer: Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_SUPERVISOR_HPP
#define H_SUPERVISOR_HPP
#pragma once

#include "constants.hpp"
#include "spsc_buffer.hpp"

#include <webots/Camera.hpp>
#include <webots/Supervisor.hpp>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <cassert>
#include <cmath>

namespace /* anonymous */ {
  std::string robot_name(bool is_red_team, std::size_t id)
  {
    return constants::DEF_ROBOT_PREFIX + (is_red_team ? "R" : "B") + std::to_string(id);
  }
} // namespace /* anonymous */


class supervisor
  : public webots::Supervisor
{
  enum { T_RED, T_BLUE }; // team
  enum { N_VIEWPOINT, N_CAMA, N_CAMB }; // node
  enum { C_CAMA, C_CAMB }; // cam

public:
  supervisor()
    : pn_cams_{getFromDef(constants::DEF_AUDVIEW), getFromDef(constants::DEF_CAMA), getFromDef(constants::DEF_CAMB)}
    , pc_cams_{getCamera(constants::NAME_CAMA), getCamera(constants::NAME_CAMB)}
  {
    constexpr auto is_null = [](const auto* p) { return !p; };

    // check if cam nodes and cam devices exist
    if(std::any_of(std::cbegin(pn_cams_), std::cend(pn_cams_), is_null)
       || std::any_of(std::cbegin(pc_cams_), std::cend(pc_cams_), is_null)) {
      throw std::runtime_error("No mandatory cam nodes exists in world");
    }

    // control visibility to cams
    control_visibility();
    enable_cameras(constants::CAM_PERIOD_MS);
  }

  bool get_deadlock_reset_flag() const
  {
    return static_cast<bool>(getSelf()->getField("deadlockReset")->getSFBool());
  }

  bool get_penalty_area_foul_flag() const
  {
    return static_cast<bool>(getSelf()->getField("penaltyAreaFoul")->getSFBool());
  }

  bool get_goal_area_foul_flag() const
  {
    return static_cast<bool>(getSelf()->getField("goalAreaFoul")->getSFBool());
  }

  std::size_t get_game_time_ms() const
  {
    const auto pf = getSelf()->getField("gameTime");

    std::size_t gt_ms = 0;

    if(pf) {
      gt_ms = static_cast<std::size_t>(pf->getSFFloat() * 1000);
    }

    if(gt_ms == 0) {
      gt_ms = constants::DEFAULT_GAME_TIME_MS;
    }

    return gt_ms / constants::PERIOD_MS * constants::PERIOD_MS;
  }

  //         name         rating  executable   data directory path
  std::tuple<std::string, double, std::string, std::string> get_team_info(bool is_red) const
  {
    const auto prefix = std::string("team") + (is_red ? "A" : "B");

    return std::make_tuple(getSelf()->getField(prefix + "Name")->getSFString(),
                           getSelf()->getField(prefix + "Rating")->getSFFloat(),
                           getSelf()->getField(prefix + "Executable")->getSFString(),
                           getSelf()->getField(prefix + "DataPath")->getSFString()
                           );
  }

  std::tuple<std::string, std::string, std::string> get_commentator_info() const
  {
    const auto prefix = std::string("commentator");

    return std::make_tuple(getSelf()->getField(prefix + "Name")->getSFString(),
                           getSelf()->getField(prefix + "Executable")->getSFString(),
                           getSelf()->getField(prefix + "DataPath")->getSFString()
                           );
  }

  std::tuple<std::string, std::string, std::string> get_reporter_info() const
  {
    const auto prefix = std::string("reporter");

    return std::make_tuple(getSelf()->getField(prefix + "Name")->getSFString(),
                           getSelf()->getField(prefix + "Executable")->getSFString(),
                           getSelf()->getField(prefix + "DataPath")->getSFString()
                           );
  }

  const unsigned char* get_image(bool is_red) const
  {
    return pc_cams_[is_red ? C_CAMA : C_CAMB]->getImage();
  }

  void reset_position()
  {
    namespace c = constants;

    const auto reset_ball_node = [&](webots::Node* pn, double x, double y, double z) {
      const double translation[] = {x, y, z};
      const double rotation[] = {0, 1, 0, 0};
      pn->getField("translation")->setSFVec3f(translation);
      pn->getField("rotation")->setSFRotation(rotation);
      pn->resetPhysics();
    };

    const auto reset_robot_node = [&](webots::Node* pn, double x, double y, double z, double th) {
      const double translation[] = {x, y, z};
      const double rotation[] = {0, 1, 0, th - c::PI / 2};
      const double lwTranslation[] = {-c::AXLE_LENGTH / 2, (-c::ROBOT_HEIGHT + 2 * c::WHEEL_RADIUS) / 2, 0};
      const double rwTranslation[] = {c::AXLE_LENGTH / 2, (-c::ROBOT_HEIGHT + 2 * c::WHEEL_RADIUS) / 2, 0};
      const double wheelRotation[] = {1, 0, 0, c::PI / 2};

      pn->getField("translation")->setSFVec3f(translation);
      pn->getField("rotation")->setSFRotation(rotation);

      pn->getField("lwTranslation")->setSFVec3f(lwTranslation);
      pn->getField("lwRotation")->setSFRotation(wheelRotation);

      pn->getField("rwTranslation")->setSFVec3f(rwTranslation);
      pn->getField("rwRotation")->setSFRotation(wheelRotation);

      pn->resetPhysics();
    };

    reset_ball_node(getFromDef(c::DEF_BALL), 0, c::BALL_RADIUS, 0);

    for(const auto& is_red : {true, false}) {
      const auto s = is_red ? 1 : -1;
      for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
        reset_robot_node(getFromDef(robot_name(is_red, id)),
                   c::ROBOT_INIT_POSTURE[id][0] * s,
                   c::ROBOT_HEIGHT / 2,
                   c::ROBOT_INIT_POSTURE[id][1] * s,
                   c::ROBOT_INIT_POSTURE[id][2] + (is_red ? 0. : c::PI));
      }
    }
  }

  std::array<double, 2> get_ball_position() const
  {
    webots::Node* pn_ball = getFromDef(constants::DEF_BALL);

    const double* position = pn_ball->getPosition();

    const double x = position[0];
    const double y = -position[2];

    return {x, y};
  }

  double get_ball_velocity() const
  {
    webots::Node* pn_ball = getFromDef(constants::DEF_BALL);

    const double* vels = pn_ball->getVelocity();
    const double x = vels[0];
    const double y = vels[1];
    const double z = vels[2];

    return std::sqrt(x * x + y * y + z * z);
  }

  //         x       y       th
  std::tuple<double, double, double> get_robot_posture(bool is_red, std::size_t id) const
  {
    webots::Node* pn_robot = getFromDef(robot_name(is_red, id));

    const double* position = pn_robot->getPosition();
    const double* orientation = pn_robot->getOrientation();

    const double x = position[0];
    const double y = -position[2];
    const double th = std::atan2(orientation[2], orientation[8]) + constants::PI / 2;

    return std::make_tuple(x, y, th);
  }

  void send_to_foulzone(bool is_red, std::size_t id)
  {
    namespace c = constants;

    webots::Node* pn_robot = getFromDef(robot_name(is_red, id));

    const auto s = is_red ? 1 : -1;
    const double translation[] = {c::ROBOT_FOUL_POSTURE[id][0] * s,
                                  c::ROBOT_HEIGHT / 2,
                                  c::ROBOT_FOUL_POSTURE[id][1] * s};
    const double rotation[] = { 0, 1, 0,
                                c::ROBOT_FOUL_POSTURE[id][2] + (is_red ? 0. : c::PI) - c::PI / 2 };
    pn_robot->getField("translation")->setSFVec3f(translation);
    pn_robot->getField("rotation")->setSFRotation(rotation);
    pn_robot->getField("customData")->setSFString("0 0");
    pn_robot->resetPhysics();
  }

  void set_linear_wheel_speed(bool is_red, std::size_t id, const std::array<double, 2>& speed)
  {
    namespace c = constants;

    webots::Node* pn_robot = getFromDef(robot_name(is_red, id));

    const auto left = std::max(std::min(speed[0], c::MAX_LINEAR_VELOCITY), -c::MAX_LINEAR_VELOCITY);
    const auto right = std::max(std::min(speed[1], c::MAX_LINEAR_VELOCITY), -c::MAX_LINEAR_VELOCITY);

    pn_robot->getField("customData")->setSFString(std::to_string(left / c::WHEEL_RADIUS) + " " +
                                            std::to_string(right / c::WHEEL_RADIUS));
  }

private: // private member functions
  void control_visibility()
  {
    using namespace constants;

    // DEF_GRASS is not visible to cam a and cam b, optional
    {
      auto* pn_grass_ = getFromDef(DEF_GRASS);
      if(pn_grass_) {
        pn_grass_->setVisibility(pn_cams_[N_CAMA], false);
        pn_grass_->setVisibility(pn_cams_[N_CAMB], false);
      }
    }

    // BALLSHAPE is visible only to viewpoint, ORANGESHAPE is to cam_a and cam_b, mandatory
    {
      auto* pn_ballshape = getFromDef(DEF_BALLSHAPE);
      auto* pn_orangeshape = getFromDef(DEF_ORANGESHAPE);
      if(!pn_ballshape || !pn_orangeshape) {
        throw std::runtime_error("No ball shape");
      }

      pn_ballshape->setVisibility(pn_cams_[N_CAMA], false);
      pn_ballshape->setVisibility(pn_cams_[N_CAMB], false);
      pn_orangeshape->setVisibility(pn_cams_[N_VIEWPOINT], false);
    }

    // Stadium is visible only to viewpoint, optional
    {
      auto* pn_stadium = getFromDef(DEF_STADIUM);
      if(pn_stadium) {
        pn_stadium->setVisibility(pn_cams_[N_CAMA], false);
        pn_stadium->setVisibility(pn_cams_[N_CAMB], false);
      }
    }

    // patches
    {
      for(const auto& team : {T_RED, T_BLUE}) {
        for(std::size_t id = 0; id < constants::NUMBER_OF_ROBOTS; ++id) {
          const auto* pn_robot = getFromDef(robot_name(team == T_RED, id));

          auto* pf_patches = pn_robot->getField("patches");

          assert(pf_patches && (pf_patches->getCount() == 3));

          auto* pn_number  = pf_patches->getMFNode(0);
          auto* pn_id_red  = pf_patches->getMFNode(1);
          auto* pn_id_blue = pf_patches->getMFNode(2);

          pn_number->setVisibility(pn_cams_[N_CAMA], false);
          pn_number->setVisibility(pn_cams_[N_CAMB], false);
          pn_id_red->setVisibility(pn_cams_[N_VIEWPOINT], false);
          pn_id_red->setVisibility(pn_cams_[N_CAMB], false);
          pn_id_blue->setVisibility(pn_cams_[N_VIEWPOINT], false);
          pn_id_blue->setVisibility(pn_cams_[N_CAMA], false);
        }
      }
    }
  }

  void enable_cameras(std::size_t period_in_ms)
  {
    for(auto* pc : pc_cams_) {
      pc->enable(period_in_ms);
    }
  }

private: // private member variables
  std::array<webots::Node*, 3> pn_cams_;
  std::array<webots::Camera*, 2> pc_cams_;
};

#endif // H_SUPERVISOR_HPP
