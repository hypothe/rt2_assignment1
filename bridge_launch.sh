#!/bin/bash

ROS_PATH=${HOME}/my_ros
ROS2_PATH=${HOME}/my_ros2
SRC_ROS1=/opt/ros/noetic/setup.bash
SRC_ROS2=/opt/ros/foxy/setup.bash

gnome-terminal --tab --title="ros1" -- bash -c "source ${SRC_ROS1}; source ${ROS_PATH}/devel/setup.bash; cd ${ROS_PATH}; roslaunch rt2_assignment1 sim_bridge.launch"

gnome-terminal --tab --title="ros1_bridge" -- bash -c "sleep 3; source ${SRC_ROS1}; source ${ROS_PATH}/devel/setup.bash; source ${SRC_ROS2}; source ${ROS2_PATH}/install/local_setup.bash; cd ${ROS2_PATH}; ros2 run ros1_bridge dynamic_bridge"

gnome-terminal --tab --title="ros2" -- bash -c "sleep 5; source ${SRC_ROS2}; source ${ROS2_PATH}/install/local_setup.bash; cd ${ROS2_PATH}; ros2 launch rt2_assignment1 sim_launch.py"
