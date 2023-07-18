#!/bin/bash

# Source the ROS setup file
source /app/devel/setup.bash

# Launch the CPU launch file
roslaunch cpu cpu_launch.launch

# Launch the CHAMP config bringup file
roslaunch champ_config bringup.launch rviz:=true

# Launch the CHAMP teleop launch file
roslaunch champ_teleop teleop.launch