#!/bin/bash

ROS_PATH=/home/my_ros
ROS2_PATH=${HOME}/my_ros2

gnome-terminal --tab --title="ros1" -- bash -c "source ${HOME}/ros.sh; cd ${ROS_PATH}; roslaunch rt2_assignment1 sim_bridge.launch"

gnome-terminal --tab --title="ros1_bridge" -- bash -c "sleep 3; source ${HOME}/ros12.sh; cd ${ROS2_PATH}; ros2 run ros1_bridge dynamic_bridge"

gnome-terminal --tab --title="ros2" -- bash -c "sleep 5; source ${HOME}/ros2.sh; cd ${ROS2_PATH}; ros2 launch rt2_assignment1 sim_launch.py"
