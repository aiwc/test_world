// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_SUPERVISOR_HPP
#define H_SUPERVISOR_HPP
#pragma once

#include "constants.hpp"
#include "spsc_buffer.hpp"

#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
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
    , pr_recv_{getReceiver(constants::NAME_RECV)}
  {
    constexpr auto is_null = [](const auto* p) { return !p; };

    // check if cam nodes and cam devices exist
    if(std::any_of(std::cbegin(pn_cams_), std::cend(pn_cams_), is_null)
       || std::any_of(std::cbegin(pc_cams_), std::cend(pc_cams_), is_null)) {
      throw std::runtime_error("No mandatory cam nodes exists in world");
    }

    // check if recv node exists
    if(pr_recv_ == NULL) {
      throw std::runtime_error("No mandatory recv node exists in world");
    }

    // control visibility to cams
    control_visibility();
    enable_cameras(constants::CAM_PERIOD_MS);
    enable_receiver(constants::RECV_PERIOD_MS);
  }

  const unsigned char* get_image(bool is_red) const
  {
    return pc_cams_[is_red ? C_CAMA : C_CAMB]->getImage();
  }

  void reset_position(constants::robot_formation red_formation, constants::robot_formation blue_formation)
  {
    namespace c = constants;

    const auto reset_ball_node = [&](webots::Node* pn, double x, double y, double z) {
      const auto f = half_passed_ ? -1 : 1;
      const double translation[] = {f * x, y, f * -z};
      const double rotation[] = {0, 1, 0, 0};
      pn->getField("translation")->setSFVec3f(translation);
      pn->getField("rotation")->setSFRotation(rotation);
      pn->resetPhysics();
    };

    const auto reset_robot_node = [&](webots::Node* pn, double x, double y, double z, double th) {
      const auto f = half_passed_ ? -1 : 1;
      const double translation[] = {f * x, y, f * -z};
      const double rotation[] = {0, 1, 0, (half_passed_ ? c::PI : 0.) + th};

      const double al = pn->getField("axleLength")->getSFFloat();
      const double h = pn->getField("height")->getSFFloat();
      const double wr = pn->getField("wheelRadius")->getSFFloat();

      const double lwTranslation[] = {-al / 2, (-h + 2 * wr) / 2, 0};
      const double rwTranslation[] = {al / 2, (-h + 2 * wr) / 2, 0};
      const double wheelRotation[] = {1, 0, 0, c::PI / 2};

      pn->getField("translation")->setSFVec3f(translation);
      pn->getField("rotation")->setSFRotation(rotation);

      pn->getField("lwTranslation")->setSFVec3f(lwTranslation);
      pn->getField("lwRotation")->setSFRotation(wheelRotation);

      pn->getField("rwTranslation")->setSFVec3f(rwTranslation);
      pn->getField("rwRotation")->setSFRotation(wheelRotation);

      pn->resetPhysics();
    };

    switch(red_formation) {
      case c::FORMATION_DEFAULT:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_DEFAULT][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_DEFAULT][1]);
        break;
      case c::FORMATION_KICKOFF:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_DEFAULT][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_DEFAULT][1]);
        break;
      case c::FORMATION_GOALKICK_A:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_GOALKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_GOALKICK][1]);
        break;
      case c::FORMATION_GOALKICK_D:
        reset_ball_node(getFromDef(c::DEF_BALL), -c::BALL_POSTURE[c::BALL_GOALKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_GOALKICK][1]);
        break;
      case c::FORMATION_CAD_AD:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CAD_DA:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CBC_AD:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, -c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CBC_DA:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, -c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CAD_AA:
        reset_ball_node(getFromDef(c::DEF_BALL), -c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, -c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CAD_DD:
        reset_ball_node(getFromDef(c::DEF_BALL), -c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, -c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CBC_AA:
        reset_ball_node(getFromDef(c::DEF_BALL), -c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_CBC_DD:
        reset_ball_node(getFromDef(c::DEF_BALL), -c::BALL_POSTURE[c::BALL_CORNERKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_CORNERKICK][1]);
        break;
      case c::FORMATION_PENALTYKICK_A:
        reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[c::BALL_PENALTYKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_PENALTYKICK][1]);
        break;
      case c::FORMATION_PENALTYKICK_D:
        reset_ball_node(getFromDef(c::DEF_BALL), -c::BALL_POSTURE[c::BALL_PENALTYKICK][0], 1.5*c::BALL_RADIUS, c::BALL_POSTURE[c::BALL_PENALTYKICK][1]);
        break;
      default:
        break;
    }

    for(const auto& is_red : {true, false}) {
      const auto s = is_red ? 1 : -1;
      auto formation = is_red ? red_formation : blue_formation;
      for(std::size_t id = 0; id < c::NUMBER_OF_ROBOTS; ++id) {
        reset_robot_node(getFromDef(robot_name(is_red, id)),
                   c::ROBOT_FORMATION[formation][id][0] * s,
                   c::ROBOT_HEIGHT[id] / 2,
                   c::ROBOT_FORMATION[formation][id][1] * s,
                   c::ROBOT_FORMATION[formation][id][2] + (is_red ? 0. : c::PI) - c::PI / 2);
      }
    }
  }

  std::array<double, 2> get_ball_position() const
  {
    const auto f = half_passed_ ? -1 : 1;
    webots::Node* pn_ball = getFromDef(constants::DEF_BALL);

    const double* position = pn_ball->getPosition();

    const double x = position[0];
    const double y = -position[2];

    return {f * x, f * y};
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
  std::tuple<double, double, double, bool> get_robot_posture(bool is_red, std::size_t id) const
  {
    namespace c = constants;

    webots::Node* pn_robot = getFromDef(robot_name(is_red, id));
    const auto f = half_passed_ ? -1 : 1;

    const double* position = pn_robot->getPosition();
    const double* orientation = pn_robot->getOrientation();

    const double x = position[0];
    const double y = -position[2];
    double th = (half_passed_ ? c::PI : 0.) + std::atan2(orientation[2], orientation[8]) + c::PI / 2;
    // Squeeze the orientation range to [-PI, PI]
    while (th > c::PI)
      th -= 2*c::PI;
    while (th < -c::PI)
      th += 2*c::PI;

    const bool  stand = orientation[4] > 0.8;

    return std::make_tuple(f * x, f * y, th, stand);
  }

  double get_distance_from_ball(bool is_red, std::size_t id) const
  {
    const auto ball = get_ball_position();
    const auto robot = get_robot_posture(is_red, id);

    const double x = ball[0] - std::get<0>(robot);
    const double y = ball[1] - std::get<1>(robot);

    return std::sqrt(x*x + y*y);
  }

  std::array<std::array<bool,constants::NUMBER_OF_ROBOTS>,2> get_robot_touch_ball() const
  {
    bool rc[2][constants::NUMBER_OF_ROBOTS];
    for (std::size_t i = 0; i < 2; i++)
      for (std::size_t j = 0; j < constants::NUMBER_OF_ROBOTS; j++)
        rc[i][j] = false;

    while (pr_recv_->getQueueLength() > 0) {
      const char *message = (const char *)pr_recv_->getData();
      for (std::size_t i = 0; i < 2; i++)
        for (std::size_t j = 0; j < constants::NUMBER_OF_ROBOTS; j++)
          if ((int)message[i+2*j] == 1) {
            rc[i][j] = true;
          }
      pr_recv_->nextPacket();
    }

    return {{{rc[0][0], rc[0][1], rc[0][2], rc[0][3], rc[0][4]}, {rc[1][0], rc[1][1], rc[1][2], rc[1][3], rc[1][4]}}};
  }

  void flush_touch_ball() const
  {
    while (pr_recv_->getQueueLength() > 0)
      pr_recv_->nextPacket();
  }

  void send_to_foulzone(bool is_red, std::size_t id)
  {
    namespace c = constants;

    webots::Node* pn = getFromDef(robot_name(is_red, id));

    const auto f = half_passed_ ? -1 : 1;
    const auto s = is_red ? 1 : -1;
    const double translation[] = {f * c::ROBOT_FOUL_POSTURE[id][0] * s,
                                  c::ROBOT_HEIGHT[id] / 2,
                                  f * -c::ROBOT_FOUL_POSTURE[id][1] * s};
    const double rotation[] = { 0, 1, 0,
                                (half_passed_ ? c::PI : 0.) + c::ROBOT_FOUL_POSTURE[id][2] + (is_red ? 0. : c::PI) - c::PI / 2 };

    const double al = pn->getField("axleLength")->getSFFloat();
    const double h = pn->getField("height")->getSFFloat();
    const double wr = pn->getField("wheelRadius")->getSFFloat();

    const double lwTranslation[] = {-al / 2, (-h + 2 * wr) / 2, 0};
    const double rwTranslation[] = {al / 2, (-h + 2 * wr) / 2, 0};
    const double wheelRotation[] = {1, 0, 0, c::PI / 2};

    pn->getField("translation")->setSFVec3f(translation);
    pn->getField("rotation")->setSFRotation(rotation);
    pn->getField("customData")->setSFString("0 0");

    pn->getField("lwTranslation")->setSFVec3f(lwTranslation);
    pn->getField("lwRotation")->setSFRotation(wheelRotation);

    pn->getField("rwTranslation")->setSFVec3f(rwTranslation);
    pn->getField("rwRotation")->setSFRotation(wheelRotation);

    pn->resetPhysics();
  }

  void return_to_field(bool is_red, std::size_t id)
  {
    namespace c = constants;

    webots::Node* pn = getFromDef(robot_name(is_red, id));

    const auto f = half_passed_ ? -1 : 1;
    const auto s = is_red ? 1 : -1;
    const double translation[] = {f * c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][0] * s,
                                  c::ROBOT_HEIGHT[id] / 2,
                                  f * -c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][1] * s};
    const double rotation[] = { 0, 1, 0,
                                (half_passed_ ? c::PI : 0.) + c::ROBOT_FORMATION[c::FORMATION_DEFAULT][id][2] + (is_red ? 0. : c::PI) - c::PI / 2 };

    const double al = pn->getField("axleLength")->getSFFloat();
    const double h = pn->getField("height")->getSFFloat();
    const double wr = pn->getField("wheelRadius")->getSFFloat();

    const double lwTranslation[] = {-al / 2, (-h + 2 * wr) / 2, 0};
    const double rwTranslation[] = {al / 2, (-h + 2 * wr) / 2, 0};
    const double wheelRotation[] = {1, 0, 0, c::PI / 2};

    pn->getField("translation")->setSFVec3f(translation);
    pn->getField("rotation")->setSFRotation(rotation);
    pn->getField("customData")->setSFString("0 0");

    pn->getField("lwTranslation")->setSFVec3f(lwTranslation);
    pn->getField("lwRotation")->setSFRotation(wheelRotation);

    pn->getField("rwTranslation")->setSFVec3f(rwTranslation);
    pn->getField("rwRotation")->setSFRotation(wheelRotation);

    pn->resetPhysics();
  }

  void relocate_ball(constants::ball_posture pos)
  {
    namespace c = constants;

    const auto reset_ball_node = [&](webots::Node* pn, double x, double y, double z) {
      const auto f = half_passed_ ? -1 : 1;
      const double translation[] = {f * x, y, f * -z};
      const double rotation[] = {0, 1, 0, 0};
      pn->getField("translation")->setSFVec3f(translation);
      pn->getField("rotation")->setSFRotation(rotation);
      pn->resetPhysics();
    };

    reset_ball_node(getFromDef(c::DEF_BALL), c::BALL_POSTURE[pos][0], 0.2, c::BALL_POSTURE[pos][1]);
  }

  void set_linear_wheel_speed(bool is_red, std::size_t id, const std::array<double, 2>& speed)
  {
    namespace c = constants;

    webots::Node* pn_robot = getFromDef(robot_name(is_red, id));

    pn_robot->getField("customData")->setSFString(std::to_string(speed[0] / c::WHEEL_RADIUS[id]) + " " +
                                            std::to_string(speed[1] / c::WHEEL_RADIUS[id]));
  }

  void mark_half_passed()
  {
    if (!half_passed_) {
      half_passed_ = true;
      const double rotation_a[] = {0, 0, 1, constants::PI};
      const double rotation_b[] = {0, 0, 1, 0};
      pn_cams_[1]->getField("rotation")->setSFRotation(rotation_a);
      pn_cams_[2]->getField("rotation")->setSFRotation(rotation_b);
    }
  }

  void mark_episode_restart()
  {
    if (half_passed_) {
      half_passed_ = false;
      const double rotation_a[] = {0, 0, 1, 0};
      const double rotation_b[] = {0, 0, 1, constants::PI};
      pn_cams_[1]->getField("rotation")->setSFRotation(rotation_a);
      pn_cams_[2]->getField("rotation")->setSFRotation(rotation_b);
    }
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
      if(!pn_orangeshape) {
        throw std::runtime_error("Missing mendatory ORANGESHAPE of BALL");
      }

      if(pn_ballshape) {
        pn_ballshape->setVisibility(pn_cams_[N_CAMA], false);
        pn_ballshape->setVisibility(pn_cams_[N_CAMB], false);
      }

      if(pn_ballshape && pn_orangeshape) {
        pn_orangeshape->setVisibility(pn_cams_[N_VIEWPOINT], false);
      }
    }

    // Stadium is visible only to viewpoint, optional
    {
      auto* pn_stadium = getFromDef(DEF_STADIUM);
      if(pn_stadium) {
        pn_stadium->setVisibility(pn_cams_[N_CAMA], false);
        pn_stadium->setVisibility(pn_cams_[N_CAMB], false);
      }
    }

    // Robot's gray cover is visible only to robots
    {
      for(const auto& team : {T_RED, T_BLUE}) {
        for(std::size_t id = 0; id < constants::NUMBER_OF_ROBOTS; ++id) {
          const auto* pn_robot = getFromDef(robot_name(team == T_RED, id));

          auto* pf_cover = pn_robot->getField("cover");

          assert(pf_cover && (pf_cover->getCount() == 1));

          auto* pn_cover  = pf_cover->getMFNode(0);

          pn_cover->setVisibility(pn_cams_[N_VIEWPOINT], false);
        }
      }
    }
    // Wall is visible only to robots
    {
      auto* pn_wall = getFromDef(DEF_WALL);
      if (pn_wall) {
        pn_wall->setVisibility(pn_cams_[N_VIEWPOINT], false);
      }
    }

    // VisualWall is visible only to viewpoint, optional
    {
      auto* pn_viswall = getFromDef(DEF_VISWALL);
      if (pn_viswall) {
        pn_viswall->setVisibility(pn_cams_[N_CAMA], false);
        pn_viswall->setVisibility(pn_cams_[N_CAMB], false);
      }
    }

    // patches
    {
      for(const auto& team : {T_RED, T_BLUE}) {
        for(std::size_t id = 0; id < constants::NUMBER_OF_ROBOTS; ++id) {
          const auto* pn_robot = getFromDef(robot_name(team == T_RED, id));

          auto* pf_patches = pn_robot->getField("patches");

          assert(pf_patches && (pf_patches->getCount() == 2 || pf_patches->getCount() == 3));

          //number patch for decoration exists
          if (pf_patches->getCount() == 3) {
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
          else { //no decorations
            auto* pn_id_red = pf_patches->getMFNode(0);
            auto* pn_id_blue = pf_patches->getMFNode(1);

            pn_id_red->setVisibility(pn_cams_[N_VIEWPOINT], true);
            pn_id_red->setVisibility(pn_cams_[N_CAMB], false);
            pn_id_blue->setVisibility(pn_cams_[N_VIEWPOINT], false);
            pn_id_blue->setVisibility(pn_cams_[N_CAMA], false);
          }
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

  void enable_receiver(std::size_t period_in_ms)
  {
    pr_recv_->enable(period_in_ms);
  }

private: // private member variables
  std::array<webots::Node*, 3> pn_cams_;
  std::array<webots::Camera*, 2> pc_cams_;
  webots::Receiver* pr_recv_;
  bool half_passed_ = false;
};

#endif // H_SUPERVISOR_HPP
