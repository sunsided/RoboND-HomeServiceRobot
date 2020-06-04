#!/usr/bin/env bash
set -euxo pipefail

echo $(dirname "$0")/../..

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm  -e  "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm  -e "source devel/setup.bash; rosrun pick_objects pick_objects_node"
