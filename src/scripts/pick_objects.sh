#!/usr/bin/env bash
set -euxo pipefail

ROOT_DIR=$(dirname "$0")/../..
cd $ROOT_DIR

# See also: http://wiki.ros.org/roslaunch/XML/arg#Passing_an_argument_via_the_command-line
TURTLEBOT_GAZEBO_WORLD_FILE=$(readlink -f "${ROOT_DIR}/src/world/world-1.world")
TURTLEBOT_GAZEBO_MAP_FILE=$(readlink -f "${ROOT_DIR}/src/map/world-1.yaml")

# See https://github.com/koalaman/shellcheck/wiki/SC2155
export TURTLEBOT_GAZEBO_WORLD_FILE
export TURTLEBOT_GAZEBO_MAP_FILE

echo "Using world: ${TURTLEBOT_GAZEBO_WORLD_FILE}"
echo "Using map:   ${TURTLEBOT_GAZEBO_MAP_FILE}"

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects_node"
