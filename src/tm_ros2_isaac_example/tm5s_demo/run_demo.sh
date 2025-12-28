#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS -------------------------------------
source /opt/ros/humble/setup.bash

echo "Running TMROS Real Robot Startup..."

if timeout 5s gnome-terminal -- bash -c "cd $SCRIPT_DIR && ./tmros_setup.sh; exec bash" ; then
    echo "TMROS fetched Real Robot successfully"
else
    echo "TMROS fetching timed out, shut down"
    exit 1
fi

sleep 2s

# Isaac Sim --------------------------------
echo "Running TMROS ISAAC Startup..."
sleep 3s

gnome-terminal -- bash -c "
$HOME/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh $SCRIPT_DIR/script/tm5s_demo.py; exec bash"

echo "Running TM ISAAC-Simulated Robot..."
