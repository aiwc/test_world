# test_world

- Downloading ZIP file through 'Download ZIP' does not work. It does not download the submodules related to the program.
- '--recursive' option is required for downloading via git command.
- Follow below to download the program directly from the browser.

#**How to download the program**

1. Go to releases (https://github.com/aiwc/test_world/releases)
2. Download the latest version of 'test_world' (zip for windows, tar.gz for linux)

#**How to install and run the program**

1. Go to Wiki page (https://github.com/aiwc/test_world/wiki)
2. Refer to Webots-related pages

#**Directory Description**

**controllers**: Contains programs for managing AI Soccer simulation system. **(Programs in this directory are managed by us and need not be modified)**

- soccer_robot: Program that changes robot wheel movements based on received data.

- supervisor: Referee program that manages AI Soccer game.

**examples**: Contains sample programs participants can refer to. **(Participants should implement AI Soccer program similar to samples in this directory)**

- common: Contains a basic interface for information handling and communication with the simulator.

- extlibs: Contains external libraries used in sample programs

- random_walk: Program that orders its team's robots to move randomly.

- skeleton-cpp: Program that orders its team's robots to move forward at maximum available velocity.

- team_a_data & team_b_data: Directory where participants' program may store some files if needed.

**extlibs**: Contains external libraries used in AI Soccer simulator. **(Files in this directory are managed by us and need not be modified)**

**protos**: Contains AI Soccer object models (robot, ball, stadium, etc.) **(Files in this directory are managed by us and need not be modified)**

**worlds**: Contains AI Soccer simulation world file **(Files in this directory can be run using Webots Robot Simulator)**

- aiwc_linux.wbt: Webots world file for linux

- aiwc_windows.wbt: Webots world file for windows

