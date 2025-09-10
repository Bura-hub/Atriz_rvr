#!/bin/bash

# Cargar el entorno de ROS
source /opt/ros/noetic/setup.bash

# Cargar el entorno del workspace
source /home/sphero/atriz_git/devel/setup.bash

# Iniciar roscore
roscore > /home/sphero/roscore.log 2>&1 &
sleep 5

# Iniciar el nodo ROS
rosrun atriz_rvr_driver Atriz_rvr_node.py > /home/sphero/rosrun.log 2>&1 &
