#!/bin/sh
export TURTLEBOT_3D_SENSOR=kinect
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/worlds/myworld.world " &
sleep 5
xterm -e "rosrun rviz rviz -d /home/workspace/catkin_ws/src/rvizConfig/home_service.rviz" &
sleep 5
xterm -e "rosrun  turtlebot gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
