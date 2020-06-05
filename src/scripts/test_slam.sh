#!/usr/bin/env bash
set -euxo pipefail

# Note that there's an error in the course materials stating that
# gmapping_demo.launch is in the gmapping package. It's not. :)
xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch"
