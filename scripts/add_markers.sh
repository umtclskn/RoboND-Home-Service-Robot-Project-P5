#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/world/myworld.world " &
sleep 10
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/map.yaml " &
sleep 10
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e " rosrun add_markers add_markers "
