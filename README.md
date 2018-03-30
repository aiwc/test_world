# AI World Cup Simulation

The AI World Cup simulation environment for usage in local PCs. Participants of [AI World Cup](http://aiworldcup.org) can download the program to develop their own algorithm for AI Soccer, AI Commentator, or AI Reporter. After development, participants of [AI World Cup](http://aiworldcup.org) can upload their algorithm online and test against others' on server provided by us. The online simulation running on our server for test matches can be used through [AI World Cup Web Simulator page](http://aiworldcup.org/web-si). As the qualifying/main rounds will be run in a same environment with the web simulation, please make sure that the developed program works in the web simulator.

- There are two ways to download the AI World Cup simulation program.
- AI World Cup simulation program requires Webots Robot Simulator. Please refer to Webots official website's [installation procedure](https://www.cyberbotics.com/doc/guide/installation-procedure) to install Webots.
- Downloading ZIP file through 'Download ZIP' will not work. It does not download the submodules related to the program.

**How to download the simulation program**

Method 1. Go to [releases](https://github.com/aiwc/test_world/releases) and download the latest version (test_world.v0.5.zip).

Method 2. Use following git command

      ## Windows only. Carriage return will invalidate simulation files.
      git config --global core.autocrlf false

      ## Both Linux and Windows
      git clone https://github.com/aiwc/test_world.git --recurse-submodules

**How to run the simulation program**

Please refer to the [Wiki pages](https://github.com/aiwc/test_world/wiki).

**Descriptions**

**controllers**: Contains programs for managing AI World Cup simulation system **(Programs in this directory are managed by us and must not be modified)**

- soccer_robot: Program that changes robot wheel movements based on received data

- supervisor: Referee program that manages AI Soccer game

**examples**: Contains sample programs participants can refer to **(Participants may implement AI programs referring to the sample programs provided in this directory)**

- common: Contains a basic interface for information handling and communication with the simulation program

- extlibs: Contains external libraries used in sample programs

- (role-prefix)_data: Directories where participants' program may write some files into if needed

- Remaining directories contain samples participants can refer to. The sample directory names follow format '(role-prefix)_(program-description)_(programming_language)'.

- (role-prefix): AI Soccer (player), AI Commentator (commentator), AI Reporter (reporter), general (Examples that may help participants from all three competitions)
- (program-description): Brief description of what the program does
- (programming_language): Programming language used in the example. Currently, we have examples for C++ (cpp) and Python (py).

**extlibs**: Contains external libraries used in AI World Cup simulation **(Files in this directory are managed by us and must not be modified)**

**plugins**: Contains a physics plugin used for ball-robot collision detection **(Files in this directory are managed by us and must not be modified)**

**protos**: Contains AI World Cup object models (robot, ball, stadium, etc.) **(Files in this directory are managed by us and must not be modified)**

**reports**: If AI World Cup simulation is run with an AI Reporter, the report will be stored in this directory after the game. The created file's name will be (reporter team name specified in config.json).txt. **(Only related to AI Reporters)**

**worlds**: Contains AI World Cup simulation world files **(Files in this directory can be run using Webots Robot Simulator)**

- aiwc.wbt: Webots world file
- aiwc_simple.wbt: A simplified version of aiwc.wbt without visual decorations (stadium, grass field, etc.). This world can be run faster than aiwc.wbt without any differences in data observed by the participant's AI program.

**config.json**: Configuration file for setting player executables, setting game duration, and setting some rules on/off for effective training. Please refer to the [Wiki page](https://github.com/aiwc/test_world/wiki/How-to-use-AI-World-Cup-simulation-program) for parameter descriptions **(Participants should modify the player information in this file to tell the simulation which program to execute)**
