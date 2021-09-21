#!/usr/bin/env bash

sudo apt update
sudo apt upgrade
sudo apt install ros-melodic-imu-sensor-controller ros-melodic-force-torque-sensor-controller ros-melodic-twist-mux ros-melodic-four-wheel-steering-controller ros-melodic-image-proc ros-melodic-range-sensor-layer ros-melodic-pointcloud-to-laserscan ros-melodic-laser-filters ros-melodic-teb-local-planner ros-melodic-gmapping ros-melodic-octomap-server ros-melodic-octomap-mapping ros-melodic-people-msgs ros-melodic-rosbridge-server ros-melodic-rospy-message-converter ros-melodic-sound-play ros-melodic-object-recognition-msgs ros-melodic-moveit-ros-planning-interface ros-melodic-move-base-msgs bison flex ros-melodic-navigation ros-melodic-moveit-planners-ompl python-scipy python-catkin-tools ros-melodic-turtlebot3* python3-catkin-pkg-modules python3-rospkg-modules python3-opencv python3-matplotlib python-rosinstall git
sudo snap install --classic code