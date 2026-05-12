#!/bin/bash

# 1. Avvia roscore in background
source /opt/ros/noetic/setup.bash
roscore &
sleep 2

# 2. Avvia il nodo ROS 1 del KUKA Med
source /ros1_ws/devel/setup.bash
# Assumendo che il nodo principale si chiami med_driver o usa il launch file se disponibile
rosrun kuka_ros_node med_driver & 
sleep 2

# 3. Avvia il bridge dinamico verso ROS 2
# Il dynamic_bridge mappa automaticamente tutti i topic compatibili
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge
