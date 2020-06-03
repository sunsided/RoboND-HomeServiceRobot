#!/usr/bin/env bash
set -euxo pipefail

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

# Note that there's an error in the course materials stating that
# gmapping_demo.launch is in the gmapping package. It's not. :)
xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
