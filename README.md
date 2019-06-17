# WCG AI Masters Simulation

The [WCG AI Masters](https://www.wcg.com/new-horizons/view/AI-Masters) simulation environment for usage in local PCs.

- There are two ways to download the WCG AI Masters simulation program.
- WCG AI Masters simulation program requires Webots Robot Simulator. Please refer to Webots official website's [installation procedure](https://www.cyberbotics.com/doc/guide/installation-procedure) to install Webots (Webots version should be R2019a-rev1).
- **Downloading the ZIP file through 'Download ZIP' will not work.** It does not download the submodules related to the program.

**How to download the simulation program**

Method 1. Go to [releases](https://github.com/wcgaimasters/wcg_ai_soccer/releases) and download the latest version.

Method 2. Use following git command

      git clone https://github.com/wcgaimasters/wcg_ai_soccer.git --recurse-submodules

**How to run the simulation program**

Please refer to the [Wiki pages](https://github.com/wcgaimasters/WCG-AI-MASTERS-Manual/wiki).

**Descriptions**

**controllers**: Contains programs for managing WCG AI Masters simulation game system **(You can modify the controllers to aid your development. However, the games at the competition will use the default controllers)**

- soccer_robot: A program that changes robot wheel movements based on received data

- supervisor: A referee program that manages AI Soccer game as whole (Participant programs communicate with this program to control the robots)

**examples**: Contains sample programs participants can refer to **(Participants may implement AI programs referring to the sample programs provided in this directory)**

- common: Contains a basic interface for information handling and communication with the simulation program

- extlibs: Contains external libraries used in sample programs

- team_a_data and team_b_data: Directories where participants' program may write some files into if needed

- Remaining directories contain samples participants can refer to.

- general_check-varaibles: A program that prints game information variables sent from the simulation program to participants program

- general_frame-skip: A program that implements framing skipping. Frame skipping is advised when your program takes more than 50 ms in each game frame in generating the output control signal

- general_image-fetch: A program that shows the game image frames using OpenCV

- player_deep-learning-play and player_deep-learning-train: Programs that implement a base skeleton for deep learning using Deep-Q-Network (DQN)

- player_random-walk: A program that simply sets robot wheel speeds to random value in each game frame

- player_rulebased-A, player_rulebased-B: Programs that implement a rule-based control of a team )'rulebased-B' is a simplified version of 'rulebased-A')

- player_skeleton: A base skeleton for the participant program

**extlibs**: Contains external libraries used in WCG AI Masters simulation

**plugins**: Contains a physics plugin used for ball-robot collision detection

**protos**: Contains WCG AI Masters object models (robot, ball, stadium, etc.)

**worlds**: Contains WCG AI Masters simulation world files **(Files in this directory can be run using Webots Robot Simulator)**

- aiwc.wbt: Webots world file

**config.json**: Configuration file for setting player executables, setting game duration, and setting some rules on/off for effective training. Please refer to the [Wiki page](https://github.com/wcgaimasters/WCG-AI-MASTERS-Manual/wiki/How-to-use-local-PC-simulation-program) for parameter descriptions **(Participants should modify the player information in this file to tell the simulation which program to execute)**
