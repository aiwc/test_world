// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_CONSTANTS_HPP
#define H_CONSTANTS_HPP
#pragma once

#include <array>
#include <string>

namespace constants {

  // M_PI needs system-specific macro. let's just define it
  constexpr double PI = 3.14159265358979323846;

  // these DEF names and names should be properly set in the world file
  const std::string DEF_AUDVIEW = "DEF_AUDVIEW";
  const std::string DEF_CAMA    = "DEF_CAMA";
  const std::string DEF_CAMB    = "DEF_CAMB";
  const std::string DEF_GRASS   = "DEF_GRASS";
  const std::string DEF_STADIUM = "DEF_STADIUM";
  const std::string NAME_CAMA   = "cam_a";
  const std::string NAME_CAMB   = "cam_b";

  // these DEF names are for dynamically created node such as ball and robots
  const std::string DEF_BALL         = "DEF_BALL";
  const std::string DEF_BALLSHAPE    = "DEF_BALLSHAPE";
  const std::string DEF_ORANGESHAPE  = "DEF_ORANGESHAPE";
  const std::string DEF_ROBOT_PREFIX = "DEF_ROBOT";

  // cams
  constexpr std::size_t RESOLUTION_X  = 640;
  constexpr std::size_t RESOLUTION_Y  = 480;
  constexpr std::size_t SUBIMAGE_NX   = 40;
  constexpr std::size_t SUBIMAGE_NY   = 40;
  constexpr std::size_t CAM_PERIOD_MS = 50; // 20 fps = 50 ms
  constexpr std::size_t ESTIMATED_SUBIMAGE_SIZE = (RESOLUTION_X / SUBIMAGE_NX) * (RESOLUTION_Y / SUBIMAGE_NY) * 4 + 100;

  // Field Dimensions
  constexpr double FIELD_LENGTH = 3.8;
  constexpr double FIELD_WIDTH  = 2.7;
  constexpr double GOAL_DEPTH   = 0.15;
  constexpr double GOAL_WIDTH   = 0.6;
  constexpr double PENALTY_AREA_DEPTH = 0.5;
  constexpr double PENALTY_AREA_WIDTH = 1.3;
  constexpr double GOAL_AREA_DEPTH = 0.2;
  constexpr double GOAL_AREA_WIDTH = 0.85;

  // Ball Dimension
  constexpr double BALL_RADIUS = 0.03;

  // Robot Dimensions
  constexpr double ROBOT_SIZE = 0.115; //DEPRECATED: will be removed on next release
  constexpr double ROBOT_HEIGHT = 0.075;
  constexpr double ROBOT_RADIUS = 0.0575;
  constexpr double AXLE_LENGTH = 0.07;
  constexpr double WHEEL_RADIUS = 0.03;

  // robot
  constexpr std::size_t NUMBER_OF_ROBOTS = 5;
  constexpr double MAX_FORCE = 0.1;
  constexpr double SLIP_NOISE = 0;//.05;
  constexpr double MAX_LINEAR_VELOCITY = 1.8;
  constexpr double BODY_MASS = 0.45;
  constexpr double WHEEL_MASS = 0.05;

  constexpr double ROBOT_INIT_POSTURE[NUMBER_OF_ROBOTS][3] = {
    // x, y, th
    {-0.4,  0.3, 0},
    {-0.4, -0.3, 0},
    {-1.0,  0.6, 0},
    {-1.0, -0.6, 0},
    {-1.8,  0.0, PI / 2},
  };

  constexpr double ROBOT_FOUL_POSTURE[NUMBER_OF_ROBOTS][3] = {
    {-1.99, 0.55, 0},
    {-1.99, 0.70, 0},
    {-1.99, 0.85, 0},
    {-1.99, 1.00, 0},
    {-1.99, 1.15, 0},
  };

  // WAMP router settings
  const std::string SERVER_IP = "127.0.0.1";
  const std::string REALM = "default";

  constexpr std::size_t KEY_LENGTH = 10;

  // Game settings
  constexpr std::size_t WAIT_READY_MS    = 30 * 1000; // ms
  constexpr std::size_t WAIT_KILL_MS     = 30 * 1000; // ms
  constexpr std::size_t WAIT_STABLE_MS   = 1  * 1000; // ms
  constexpr std::size_t WAIT_GOAL_MS     = 3  * 1000; // ms
  constexpr std::size_t WAIT_END_MS      = 3  * 1000; // ms
  constexpr std::size_t DEFAULT_GAME_TIME_MS     = 300 * 1000; // ms
  constexpr std::size_t PERIOD_MS        = 50; // ms
  constexpr std::size_t FOUL_PA_DURATION_MS = 5 * 1000; // ms
  constexpr std::size_t FOUL_GA_DURATION_MS = 5 * 1000; // ms
  constexpr double      FOUL_PA_THRESHOLD = 4.; // number of robots in penalty area
  constexpr double      FOUL_GA_THRESHOLD = 2.; // number of robots in goal area
  constexpr double      DEADLOCK_DURATION_MS  = 5 * 1000; // ms
  constexpr double      DEADLOCK_THRESHOLD = 0.1; // m/s

  constexpr std::size_t NUM_COMMENTS = 10;

  constexpr std::size_t MSG_MAX_SIZE = 90000; // bytes

  enum reason_code {
    NONE = 0,
    GAME_START = 1,
    SCORE_RED_TEAM = 2,
    SCORE_BLUE_TEAM = 3,
    GAME_END = 4,
    DEADLOCK = 5,
  };

  constexpr std::array<std::size_t, 5> CODEWORDS = {
    0b000000000,
    0b000011111,
    0b011100011,
    0b101101100,
    0b110110101,
  };

} // namespace constants

#endif // H_CONSTANTS_HPP
