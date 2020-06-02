
# RoboND-Home-Service-Robot-Project-P5
The goal of this project is to program a robot that can autonomously map and environment and navigate to pick up and dropped virtual objects with ROS Kinetic & Gazebo.  Solution for Udacity Robotics Software Engineer Nanodegree Program


sudo apt-get update && apt-get upgrade
sudo apt-get install xterm

$ mkdir -p /home/workspace/catkin_ws/src
$ cd /home/workspace/catkin_ws/src
$ catkin_init_workspace

$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/turtlebot/turtlebot_simulator
$ git clone https://github.com/turtlebot/turtlebot
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
$ git clone https://github.com/turtlebot/turtlebot_interactions.git
$ git clone https://github.com/ros-perception/slam_gmapping
$ rosdep install gmapping
$ cd ..
$ catkin_make
$ source devel/setup.bash
