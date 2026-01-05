#!/bin/bash

# Clean environment from snap interference
unset GTK_PATH
unset GTK_EXE_PREFIX  
unset GIO_MODULE_DIR
unset LOCPATH
unset GTK_IM_MODULE_FILE

# Deactivate conda if active
if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
    conda deactivate 2>/dev/null || true
fi

# Source ROS2 workspace
source install/setup.bash

# Run color tracker
python3 color_tracker.py --ros-args -p use_sim_time:=true "$@"
