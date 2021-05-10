#!/bin/bash

gnome-terminal --tab --title="ros1" -- bash -c "source ${HOME}/ros.sh; cd /home/my_ros; roslaunch rt2_assignment1 sim_bridge.launch"

gnome-terminal --tab --title="ros1_bridge" -- bash -c "sleep 3; source ${HOME}/ros12.sh; cd ${HOME}/my_ros2; ros2 run ros1_bridge dynamic_bridge"

gnome-terminal --tab --title="ros2" -- bash -c "sleep 5; source ${HOME}/ros2.sh; cd ${HOME}/my_ros2; ros2 launch rt2_assignment1 sim_launch.py"
