# test_world

- Downloading ZIP file through 'Download ZIP' will not work. It does not download the submodules related to the program.
- To use git command, use '--recurse-submodules' option to clone submodules along with the main repository (git clone https://(address) --recurse-submodules).
- Follow below to download the program directly from the web browser.

**How to download the program**

1. Go to releases (https://github.com/aiwc/test_world/releases)
2. Download the latest version of 'test_world' (zip for windows, tar.gz for linux)

**How to install and run the program**

1. Go to Wiki page (https://github.com/aiwc/test_world/wiki)
2. Refer to Webots-related pages

**Directory Description**

**controllers**: Contains programs for managing AI Soccer simulation system. **(Programs in this directory are managed by us and need not be modified)**

- soccer_robot: Program that changes robot wheel movements based on received data.

- supervisor: Referee program that manages AI Soccer game.

**examples**: Contains sample programs participants can refer to. **(Participants can implement AI Soccer program in similar way with the sample programs in this directory)**

- common: Contains a basic interface for information handling and communication with the simulator.

- extlibs: Contains external libraries used in sample programs.

- (role-prefix)_data: Directories where participants' program may write some files into if needed.

- Remaining directories contain samples participants can refer to. The sample directory names follow format '(role-prefix)_(program-description)_(programming_language)'.

- (role-prefix): player (AI Soccer), commentator (AI Commentator), reporter (AI Reporter), general (Examples that may help participants from all three competitions).
- (program-description): Breif decsription of what example the program is.
- (programming_language): Programming language used to develop the example (C++ or Python).

**extlibs**: Contains external libraries used in AI Soccer simulator. **(Files in this directory are managed by us and must not be modified)**

**protos**: Contains AI Soccer object models (robot, ball, stadium, etc.) **(Files in this directory are managed by us and need not be modified)**

**worlds**: Contains AI Soccer simulation world file **(Files in this directory can be run using Webots Robot Simulator)**

- aiwc_linux.wbt: Webots world file for linux

- aiwc_windows.wbt: Webots world file for windows

