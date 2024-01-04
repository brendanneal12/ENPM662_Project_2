UMD ENPM662 Project 2

Student Information:
Brendan Neal
Directory ID: bneal12.
UID: 119471128.

Adam Lobo
Directory ID: alobo.
UID: 115806078.

Project Information:
Goal: Design and control a mobile manipulator to clear tables.

Package Name: busbot

Recommended IDE: Visual Studio Code

Python Version: 3

PLEASE NOTE THE VIDEOS ARE LINKS IN THE REPORT.

How to Run Code:
SETUP:
1. Create a ROS2 Workspace
2. Download busbot to the src folder.
3. Build and source the package.

CONTROLLER SCRIPT:
1. In one terminal, build, source, then run "ros2 launch busbot gazebo.launch.py"
2. Open another terminal, cd to the workspace, source, then run "ros2 run busbot controller.py"
3. The motion planning phase will begins. This plans all the joint angles needed to complete 4 trajectories.
4. Once the planning (~25-30 seconds) is done, the robot control will begin. Observe the robot drive forward, then execute 4 trajectories and pick up the cup on the table.
5. Wait 2 seconds after seeing "Finish Trajectory 4!"
6. Ctrl+C in that terminal to drop the cup into the basket.
7. PROGRAM END.

VALIDATION SCRIPTS:
1. Open package in an IDE
2. Navigate to scripts/validation scripts
3. Run the forward kinematics and ivnerse kinematics scripts to get a visual representation of our fkin and ikin working properly.


