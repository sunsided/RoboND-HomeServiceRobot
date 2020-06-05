#!/usr/bin/env bash
set -euxo pipefail

ROOT_DIR=$(dirname "$0")/../..
cd $ROOT_DIR

# See also: http://wiki.ros.org/roslaunch/XML/arg#Passing_an_argument_via_the_command-line
# Note https://github.com/koalaman/shellcheck/wiki/SC2155 - doesn't work on Udacity workspaces though.
export TURTLEBOT_GAZEBO_WORLD_FILE=$(readlink -f "${ROOT_DIR}/src/world/world-1.world")
export TURTLEBOT_GAZEBO_MAP_FILE=$(readlink -f "${ROOT_DIR}/src/map/world-1.yaml")

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch"
