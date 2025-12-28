#!/bin/bash

echo "Running TMROS Driver..."

cd $HOME/tm2_ros2_ws

source ./install/setup.bash

ros2 launch tm5s_moveit_config tm5s_run_move_group.launch.py
