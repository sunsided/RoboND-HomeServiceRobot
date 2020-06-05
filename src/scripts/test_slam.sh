#!/usr/bin/env bash
set -euxo pipefail

ROOT_DIR=$(dirname "$0")/../..
cd $ROOT_DIR

# See also: http://wiki.ros.org/roslaunch/XML/arg#Passing_an_argument_via_the_command-line
TURTLEBOT_GAZEBO_WORLD_FILE=$(readlink -f "${ROOT_DIR}/src/world/world-1.world")

# See https://github.com/koalaman/shellcheck/wiki/SC2155
export TURTLEBOT_GAZEBO_WORLD_FILE

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

# Note that there's an error in the course materials stating that
# gmapping_demo.launch is in the gmapping package. It's not. :)
xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
