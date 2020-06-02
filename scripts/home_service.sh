#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/world/myworld.world " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/map.yaml " &
sleep 5
xterm -e " rosrun rviz rviz  " &
sleep 15
xterm -e "rosrun add_markers add_markers " &
sleep 5
xterm -e "rosrun pick_objects pick_objects"
