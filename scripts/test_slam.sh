#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/worlds/myworld.world " &
sleep 5
xterm -e "rosrun rviz rviz " &
sleep 5
xterm -e "rosrun gmapping slam_gmapping" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
