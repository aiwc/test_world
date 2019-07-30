DEF_AUDVIEW = 'DEF_AUDVIEW'
DEF_CAMA = 'DEF_CAMA'
DEF_CAMB = 'DEF_CAMB'
DEF_GRASS = 'DEF_GRASS'
DEF_STADIUM = 'DEF_STADIUM'
DEF_WALL = 'DEF_WALL'
DEF_VISWALL = 'DEF_VISWALL'
NAME_CAMA = 'cam_a'
NAME_CAMB = 'cam_b'
NAME_RECV = 'recv'

# these DEF names are for dynamically created node such as ball and robots
DEF_BALL = 'DEF_BALL'
DEF_BALLSHAPE = 'DEF_BALLSHAPE'
DEF_ORANGESHAPE = 'DEF_ORANGESHAPE'
DEF_ROBOT_PREFIX = 'DEF_ROBOT'

# cams
RESOLUTION_X = 640
RESOLUTION_Y = 480
SUBIMAGE_NX = 40
SUBIMAGE_NY = 40
CAM_PERIOD_MS = 50  # 20 fps = 50 ms
RECV_PERIOD_MS = 50
ESTIMATED_SUBIMAGE_SIZE = (RESOLUTION_X / SUBIMAGE_NX) * (RESOLUTION_Y / SUBIMAGE_NY) * 4 + 100

#   Field Dimensions
FIELD_LENGTH = 7.8
FIELD_WIDTH = 4.65
GOAL_DEPTH = 0.45
GOAL_WIDTH = 1.0
PENALTY_AREA_DEPTH = 0.9
PENALTY_AREA_WIDTH = 1.8
GOAL_AREA_DEPTH = 0.4
GOAL_AREA_WIDTH = 1.3
WALL_THICKNESS = 0.025
CORNER_LENGTH = 0.1

#   Ball Dimension
BALL_RADIUS = 0.04
BALL_MASS = 0.0184

#  Robot Specifications
NUMBER_OF_ROBOTS = 5
ROBOT_SIZE = [0.15, 0.15, 0.15, 0.15, 0.15]
ROBOT_HEIGHT = [0.09, 0.09, 0.09, 0.09, 0.09]
AXLE_LENGTH = [0.14, 0.14, 0.14, 0.14, 0.14]
ROBOT_BODY_MASS = [2.5, 2.0, 2.0, 1.5, 1.5]
WHEEL_RADIUS = [0.04, 0.04, 0.04, 0.04, 0.04]
WHEEL_MASS = [0.15, 0.10, 0.10, 0.10, 0.10]
MAX_LINEAR_VELOCITY = [1.8, 2.1, 2.1, 2.55, 2.55]
MAX_TORQUE = [0.8, 1.2, 1.2, 0.4, 0.4]
PI = 3.14159265358979323846
ROBOT_FORMATION = [
  #  x, y, th - Default Formation
  [[-3.8,   0.0, PI / 2],
   [-2.25,  1.0, 0],
   [-2.25, -1.0, 0],
   [-0.65,  0.3, 0],
   [-0.65, -0.3, 0]],
  #  x, y, th - Kickoff Formation
  [[-3.8,   0.0, PI / 2],
   [-2.25,  1.0, 0],
   [-2.25, -1.0, 0],
   [-0.9,  0,   0],
   [0.4,   0,   PI]],
  #  x, y, th - Goalkick-Attack Formation
  [[-3.8,   0.0, 0],
   [-2.5,    0.45, 0],
   [-2.5,   -0.45, 0],
   [-1.5,     0.8, 0],
   [-1.5,    -0.8, 0]],
  #  x, y, th - Goalkick-Defense Formation
  [[-3.8,   0.0, PI / 2],
   [-0.5,   0.8, 0],
   [-0.5,  -0.8, 0],
   [0.5,    0.45, 0],
   [0.5,   -0.45, 0]],
  #  x, y, th - Corner AD - Attack-Attack Formation
  [[-3.8,   0.0, PI / 2],
   [2.25, -1.0, PI / 2],
   [3.25, -1.0, PI / 2],
   [2.25,  0.0,      0],
   [2.75, -2.0, PI / 2]],
  #  x, y, th - Corner AD - Attack-Defense Formation
  [[-3.8,   0.0, PI / 2],
   [-3.25,  0.5, PI / 2],
   [-3.25, -0.5, PI / 2],
   [-2.25,  0.5, PI / 2],
   [-2.25, -0.5, PI / 2]],
  #  x, y, th - Corner AD - Defense-Attack Formation
  [[-3.8,   0.0, PI / 2],
   [-2.25,  1.0, 3*PI / 2],
   [-3.25,  1.0, 3*PI / 2],
   [-3.25,  0.0,        0],
   [-2.75,  2.0, 3*PI / 2]],
  #  x, y, th - Corner AD - Defense-Defense Formation
  [[-3.8,   0.0, PI / 2],
   [-1.5,   0.45, 0],
   [-1.5,  -0.45, 0],
   [-0.5,   0.8, 0],
   [-0.5,  -0.8, 0]],
  #  x, y, th - Corner BC - Attack-Attack Formation
  [[-3.8,   0.0, PI / 2],
   [3.25,  1.0, 3*PI / 2],
   [2.25,  1.0, 3*PI / 2],
   [2.25,  0.0,        0],
   [2.75,  2.0, 3*PI / 2]],
  #  x, y, th - Corner BC - Attack-Defense Formation
  [[-3.8,   0.0, PI / 2],
   [-3.25,  0.5, 3*PI / 2],
   [-3.25, -0.5, 3*PI / 2],
   [-2.25,  0.5, 3*PI / 2],
   [-2.25, -0.5, 3*PI / 2]],
  #  x, y, th - Corner BC - Defense-Attack Formation
  [[-3.8,   0.0, PI / 2],
   [-3.25, -1.0, PI / 2],
   [-2.25, -1.0, PI / 2],
   [-3.25,  0.0, 0],
   [-2.75, -2.0, PI / 2]],
  #  x, y, th - Corner BC - Defense-Defense Formation
  [[-3.8,   0.0, PI / 2],
   [-1.5,   0.45, 0],
   [-1.5,  -0.45, 0],
   [-0.5,   0.8, 0],
   [-0.5,  -0.8, 0]],
  #  x, y, th - Penaltykick - Attack Formation
  [[-3.8,   0.0, PI / 2],
   [0.5,  -0.8, 0],
   [1.0,  -0.8, 0],
   [1.5,  -0.8, 0],
   [2.0,   0.0, 0]],
  #  x, y, th - Penaltykick - Defense Formation
  [[-3.8,   0.0,  PI / 2],
   [-1.5,  -0.8,  PI / 2],
   [-1.5,  -1.05, PI / 2],
   [-1.25, -0.8,  PI / 2],
   [-1.25, -1.05, PI / 2]]
]
ROBOT_DEFAULT = 0
ROBOT_KICKOFF = 1
ROBOT_GOALKICK_A = 2
ROBOT_GOALKICK_D = 3
ROBOT_CAD_AA = 4
ROBOT_CAD_AD = 5
ROBOT_CAD_DA = 6
ROBOT_CAD_DD = 7
ROBOT_CBC_AA = 8
ROBOT_CBC_AD = 9
ROBOT_CBC_DA = 10
ROBOT_CBC_DD = 11
ROBOT_PENALTYKICK_A = 12
ROBOT_PENALTYKICK_D = 13

ROBOT_FOUL_POSTURE = [
 [-4.05, -0.85, 0],
 [-4.05, -1.15, 0],
 [-4.05, -1.45, 0],
 [-4.05, -1.75, 0],
 [-4.05, -2.05, 0],
]
BALL_POSTURE = [
 [0.0,  0.0],
 [-3.25,  0.0],
 [-1.5,  1.0],
 [-1.5, -1.0],
 [1.5,  1.0],
 [1.5, -1.0],
 [-2.75,  1.5],
 [2.95,  0.0],
]
BALL_DEFAULT = 0
BALL_GOALKICK = 1
BALL_RELOCATION_A = 2
BALL_RELOCATION_B = 3
BALL_RELOCATION_C = 4
BALL_RELOCATION_D = 5
BALL_CORNERKICK = 6
BALL_PENALTYKICK = 7

# Server settings
SERVER_IP = '127.0.0.1'
SERVER_PORT = 5000

KEY_LENGTH = 10

# Game settings
WAIT_READY_MS = 30 * 1000  # ms
WAIT_KILL_MS = 30 * 1000  # ms
WAIT_STABLE_MS = 1 * 1000  # ms
WAIT_GOAL_MS = 3 * 1000  # ms
WAIT_END_MS = 3 * 1000  # ms
DEFAULT_GAME_TIME_MS = 300 * 1000  # ms
PERIOD_MS = 50  # ms
PA_THRESHOLD_A = 2  # penalty area robot limit for attacking team
PA_THRESHOLD_D = 3  # penalty area robot limit for defending team
DEADLOCK_SENTOUT_NUMBER = 2  # number of robots sent out when a deadlock happens
SENTOUT_DURATION_MS = 5 * 1000  # ms
FALL_TIME_MS = 3 * 1000  # ms
DEADLOCK_DURATION_MS = 4 * 1000  # ms
DEADLOCK_THRESHOLD = 0.4  # m/s
KICKOFF_TIME_LIMIT_MS = 3 * 1000  # ms
KICKOFF_BORDER = 0.5  # m
GOALKICK_TIME_LIMIT_MS = 3 * 1000  # ms
CORNERKICK_TIME_LIMIT_MS = 3 * 1000  # ms
PENALTYKICK_TIME_LIMIT_MS = 3 * 1000  # ms
NUM_COMMENTS = 3
MSG_MAX_SIZE = 90000  # bytes
NONE = 0
GAME_START = 1
SCORE_RED_TEAM = 2
SCORE_BLUE_TEAM = 3
GAME_END = 4
DEADLOCK = 5
GOALKICK = 6
CORNERKICK = 7
PENALTYKICK = 8
HALFTIME = 9
EPISODE_END = 10
CODEWORDS = [0b000000000, 0b000011111, 0b011100011, 0b101101100, 0b110110101]
