// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_CONSTANTS_HPP
#define H_CONSTANTS_HPP
#pragma once

#include <array>
#include <cmath>
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
  const std::string DEF_WALL    = "DEF_WALL";
  const std::string DEF_VISWALL = "DEF_VISWALL";
  const std::string NAME_CAMA   = "cam_a";
  const std::string NAME_CAMB   = "cam_b";
  const std::string NAME_RECV   = "recv";

  // these DEF names are for dynamically created node such as ball and robots
  const std::string DEF_BALL         = "DEF_BALL";
  const std::string DEF_BALLSHAPE    = "DEF_BALLSHAPE";
  const std::string DEF_ORANGESHAPE  = "DEF_ORANGESHAPE";
  const std::string DEF_ROBOT_PREFIX = "DEF_ROBOT";

  // cams
  constexpr std::size_t RESOLUTION_X   = 640;
  constexpr std::size_t RESOLUTION_Y   = 480;
  constexpr std::size_t SUBIMAGE_NX    = 40;
  constexpr std::size_t SUBIMAGE_NY    = 40;
  constexpr std::size_t CAM_PERIOD_MS  = 50; // 20 fps = 50 ms
  constexpr std::size_t RECV_PERIOD_MS = 50;
  constexpr std::size_t ESTIMATED_SUBIMAGE_SIZE = (RESOLUTION_X / SUBIMAGE_NX) * (RESOLUTION_Y / SUBIMAGE_NY) * 4 + 100;

  // Field Dimensions
  constexpr double FIELD_LENGTH = 7.8;
  constexpr double FIELD_WIDTH  = 4.65;
  constexpr double GOAL_DEPTH   = 0.45;
  constexpr double GOAL_WIDTH   = 1.0;
  constexpr double PENALTY_AREA_DEPTH = 0.9;
  constexpr double PENALTY_AREA_WIDTH = 1.8;
  constexpr double GOAL_AREA_DEPTH = 0.4;
  constexpr double GOAL_AREA_WIDTH = 1.3;
  constexpr double WALL_THICKNESS = 0.025;
  constexpr double CORNER_LENGTH = 0.1;

  // Ball Dimension
  constexpr double BALL_RADIUS = 0.04;
  constexpr double BALL_MASS = 0.0184;

  // Robot Specifications
  constexpr std::size_t NUMBER_OF_ROBOTS = 5;
  constexpr std::array<double, 5> ROBOT_SIZE = {0.15, 0.15, 0.15, 0.15, 0.15};
  constexpr std::array<double, 5> ROBOT_HEIGHT = {0.09, 0.09, 0.09, 0.09, 0.09};
  constexpr std::array<double, 5> AXLE_LENGTH = {0.14, 0.14, 0.14, 0.14, 0.14};
  constexpr std::array<double, 5> ROBOT_BODY_MASS = {2.5, 2.0, 2.0, 1.5, 1.5};

  constexpr std::array<double, 5> WHEEL_RADIUS = {0.04, 0.04, 0.04, 0.04, 0.04};
  constexpr std::array<double, 5> WHEEL_MASS = {0.15, 0.10, 0.10, 0.10, 0.10};

  constexpr std::array<double, 5> MAX_LINEAR_VELOCITY = {1.8, 2.1, 2.1, 2.55, 2.55};
  constexpr std::array<double, 5> MAX_TORQUE = {0.8, 1.2, 1.2, 0.4, 0.4};

  constexpr double ROBOT_FORMATION[14][NUMBER_OF_ROBOTS][3] = {
    // x, y, th - Default Formation
   {{-3.8,   0.0, PI / 2},
    {-2.25,  1.0, 0},
    {-2.25, -1.0, 0},
    {-0.65,  0.3, 0},
    {-0.65, -0.3, 0},},
    // x, y, th - Kickoff Formation
   {{-3.8,   0.0, PI / 2},
    {-2.25,  1.0, 0},
    {-2.25, -1.0, 0},
    {-0.9,  0,   0},
    { 0.4,   0,   PI},},
    // x, y, th - Goalkick-Attack Formation
   {{-3.8,   0.0, 0},
    {-2.5,    0.45, 0},
    {-2.5,   -0.45, 0},
    {-1.5,     0.8, 0},
    {-1.5,    -0.8, 0},},
    // x, y, th - Goalkick-Defense Formation
   {{-3.8,   0.0, PI / 2},
    {-0.5,   0.8, 0},
    {-0.5,  -0.8, 0},
    { 0.5,   0.45, 0},
    { 0.5,  -0.45, 0},},
    // x, y, th - Corner AD - Attack-Attack Formation
   {{-3.8,   0.0, PI / 2},
    { 2.25, -1.0, PI / 2},
    { 3.25, -1.0, PI / 2},
    { 2.25,  0.0,      0},
    { 2.75, -2.0, PI / 2},},
    // x, y, th - Corner AD - Attack-Defense Formation
   {{-3.8,   0.0, PI / 2},
    {-3.25,  0.5, PI / 2},
    {-3.25, -0.5, PI / 2},
    {-2.25,  0.5, PI / 2},
    {-2.25, -0.5, PI / 2},},
    // x, y, th - Corner AD - Defense-Attack Formation
   {{-3.8,   0.0, PI / 2},
    {-2.25,  1.0, 3*PI / 2},
    {-3.25,  1.0, 3*PI / 2},
    {-3.25,  0.0,        0},
    {-2.75,  2.0, 3*PI / 2},},
    // x, y, th - Corner AD - Defense-Defense Formation
   {{-3.8,   0.0, PI / 2},
    {-1.5,   0.45, 0},
    {-1.5,  -0.45, 0},
    {-0.5,   0.8, 0},
    {-0.5,  -0.8, 0},},
    // x, y, th - Corner BC - Attack-Attack Formation
   {{-3.8,   0.0, PI / 2},
    { 3.25,  1.0, 3*PI / 2},
    { 2.25,  1.0, 3*PI / 2},
    { 2.25,  0.0,        0},
    { 2.75,  2.0, 3*PI / 2},},
    // x, y, th - Corner BC - Attack-Defense Formation
   {{-3.8,   0.0, PI / 2},
    {-3.25,  0.5, 3*PI / 2},
    {-3.25, -0.5, 3*PI / 2},
    {-2.25,  0.5, 3*PI / 2},
    {-2.25, -0.5, 3*PI / 2},},
    // x, y, th - Corner BC - Defense-Attack Formation
   {{-3.8,   0.0, PI / 2},
    {-3.25, -1.0, PI / 2},
    {-2.25, -1.0, PI / 2},
    {-3.25,  0.0, 0     },
    {-2.75, -2.0, PI / 2},},
    // x, y, th - Corner BC - Defense-Defense Formation
   {{-3.8,   0.0, PI / 2},
    {-1.5,   0.45, 0},
    {-1.5,  -0.45, 0},
    {-0.5,   0.8, 0},
    {-0.5,  -0.8, 0},},
    // x, y, th - Penaltykick - Attack Formation
   {{-3.8,   0.0, PI / 2},
    { 0.5,  -0.8, 0},
    { 1.0,  -0.8, 0},
    { 1.5,  -0.8, 0},
    { 2.0,   0.0, 0},},
    // x, y, th - Penaltykick - Defense Formation
   {{-3.8,   0.0,  PI / 2},
    {-1.5,  -0.8,  PI / 2},
    {-1.5,  -1.05, PI / 2},
    {-1.25, -0.8,  PI / 2},
    {-1.25, -1.05, PI / 2},},
  };

  constexpr double ROBOT_FOUL_POSTURE[NUMBER_OF_ROBOTS][3] = {
    {-4.05, -0.85, 0},
    {-4.05, -1.15, 0},
    {-4.05, -1.45, 0},
    {-4.05, -1.75, 0},
    {-4.05, -2.05, 0},
  };

  constexpr double BALL_POSTURE[8][2] = {
    {  0.0,  0.0},
    {-3.25,  0.0},
    { -1.5,  1.0},
    { -1.5, -1.0},
    {  1.5,  1.0},
    {  1.5, -1.0},
    {-2.75,  1.5},
    { 2.95,  0.0},
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
  constexpr std::size_t PA_THRESHOLD_A = 2; // penalty area robot limit for attacking team
  constexpr std::size_t PA_THRESHOLD_D = 3; // penalty area robot limit for defending team
  constexpr std::size_t DEADLOCK_SENTOUT_NUMBER = 2; // number of robots sent out when a deadlock happens
  constexpr std::size_t SENTOUT_DURATION_MS = 5 * 1000; // ms
  constexpr std::size_t FALL_TIME_MS = 3 * 1000; // ms
  constexpr std::size_t DEADLOCK_DURATION_MS  = 4 * 1000; // ms
  constexpr double      DEADLOCK_THRESHOLD = 0.4; // m/s
  constexpr std::size_t KICKOFF_TIME_LIMIT_MS = 3 * 1000; // ms
  constexpr double      KICKOFF_BORDER = 0.5; // m
  constexpr std::size_t GOALKICK_TIME_LIMIT_MS = 3 * 1000; // ms
  constexpr std::size_t CORNERKICK_TIME_LIMIT_MS = 3 * 1000; // ms
  constexpr std::size_t PENALTYKICK_TIME_LIMIT_MS = 3 * 1000; // ms
  constexpr std::size_t IOPA_TIME_LIMIT_MS = 1 * 1000; // ms
  constexpr std::size_t GK_NIPA_TIME_LIMIT_MS = 3 * 1000; // ms

  constexpr std::size_t NUM_COMMENTS = 3;

  constexpr std::size_t MSG_MAX_SIZE = 90000; // bytes

  enum reason_code {
    NONE = 0,
    GAME_START = 1,
    SCORE_RED_TEAM = 2,
    SCORE_BLUE_TEAM = 3,
    GAME_END = 4,
    DEADLOCK = 5,
    GOALKICK = 6,
    CORNERKICK = 7,
    PENALTYKICK = 8,
    HALFTIME = 9,
    EPISODE_END = 10,
  };

  enum game_state {
    STATE_DEFAULT = 0,
    STATE_KICKOFF = 1,
    STATE_GOALKICK = 2,
    STATE_CORNERKICK = 3,
    STATE_PENALTYKICK = 4,
  };

  enum robot_formation {
    FORMATION_DEFAULT = 0,
    FORMATION_KICKOFF = 1,
    FORMATION_GOALKICK_A = 2,
    FORMATION_GOALKICK_D = 3,
    FORMATION_CAD_AA = 4,
    FORMATION_CAD_AD = 5,
    FORMATION_CAD_DA = 6,
    FORMATION_CAD_DD = 7,
    FORMATION_CBC_AA = 8,
    FORMATION_CBC_AD = 9,
    FORMATION_CBC_DA = 10,
    FORMATION_CBC_DD = 11,
    FORMATION_PENALTYKICK_A = 12,
    FORMATION_PENALTYKICK_D = 13,
  };

  enum ball_posture {
    BALL_DEFAULT = 0,
    BALL_GOALKICK = 1,
    BALL_RELOCATION_A = 2,
    BALL_RELOCATION_B = 3,
    BALL_RELOCATION_C = 4,
    BALL_RELOCATION_D = 5,
    BALL_CORNERKICK = 6,
    BALL_PENALTYKICK = 7,
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
