# AI World Cup Simulation

The AI World Cup simulation environment. Participants of [AI World Cup](http://aiworldcup.org/) can download the program to develop their own algorithm for AI Soccer, AI Commentator, or AI Reporter. After development, participants of AI World Cup can submit their programs to [aiwc@rit.kaist.ac.kr](aiwc@rit.kaist.ac.kr). We will check if your program runs correctly on our server where the qualifying/main rounds will be run and reply back as soon as possible. Regarding the deadlines, please refer to our [AI World Cup Website](http://aiworldcup.org/).

- There are two ways to download the AI World Cup simulation program.
- AI World Cup simulation program requires Webots Robot Simulator. Please refer to Webots official website's [installation procedure](https://www.cyberbotics.com/doc/guide/installation-procedure) to install Webots (Webots version should be R2019b).
- **Downloading the ZIP file through 'Download ZIP' will not work.** It does not download the submodules related to the program.

**How to download the simulation program**

Method 1. Go to [releases](https://github.com/aiwc/test_world/releases) and download the latest version.

Method 2. Use following git command

      git clone https://github.com/aiwc/test_world.git --recurse-submodules

**How to run the simulation program**

Please refer to the [Wiki pages](https://github.com/aiwc/test_world/wiki).

**Descriptions**

**controllers**: Contains programs for managing AI World Cup simulation game system **(You can modify the controllers to aid your development. However, the games at the competition will use the default controllers)**

- soccer_robot: A program that changes robot wheel movements based on received data

- supervisor: A referee program that manages AI World Cup games as whole (Participant programs communicate with this program to control the robots/make comments/submit a report)

**examples**: Contains sample programs participants can refer to **(Participants may implement AI programs referring to the sample programs provided in this directory)**

- common: Contains a basic interface for C++ information handling and communication with the simulation program

- extlibs: Contains external libraries used in sample programs

- team_a_data, team_b_data, commentator_data, and reporter_data: Directories where participants' program may write some files into if needed

- Remaining directories contain samples participants can refer to.

- general_check-variables: A program that prints game information variables sent from the simulation program to participants program

- general_frame-skip: A program that implements framing skipping. Frame skipping is advised when your program takes more than 50 ms in each game frame in generating the output control signal

- general_image-fetch: A program that shows the game image frames using OpenCV

- player_deep-learning-play and player_deep-learning-train: Programs that implement a base skeleton for AI Soccer deep learning using Deep-Q-Network (DQN)

- player_random-walk: An AI Soccer program that simply sets robot wheel speeds to random value in each game frame

- player_rulebased-A, player_rulebased-B: Programs that implement a rule-based control of a team in AI Soccer ('rulebased-B' is a simplified version of 'rulebased-A')

- player_skeleton: A base skeleton for AI Soccer program

- commentator_skeleton: A base skeleton for AI Commentator program

- reporter_skeleton: A base skeleton for AI Reporter program

**extlibs**: Contains external libraries used in AI World Cup simulation

**plugins**: Contains a physics plugin used for ball-robot collision detection

**protos**: Contains AI World Cup object models (robot, ball, stadium, etc.)

**worlds**: Contains AI World Cup simulation world files **(Files in this directory can be run using Webots Robot Simulator)**

- aiwc.wbt: Webots world file

**config.json**: Configuration file for setting player executables, setting game duration, and setting some rules on/off for effective training. Please refer to the [Wiki page](https://github.com/aiwc/test_world/wiki/How-to-use-AI-World-Cup-simulation-program) for parameter descriptions **(Participants should modify the player information in this file to tell the simulation which program to execute)**
