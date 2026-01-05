#!/bin/bash

# Clean environment from snap interference
unset GTK_PATH
unset GTK_EXE_PREFIX  
unset GIO_MODULE_DIR
unset LOCPATH
unset GTK_IM_MODULE_FILE

# Force deactivate conda by unsetting all conda variables
unset CONDA_DEFAULT_ENV
unset CONDA_EXE
unset CONDA_PREFIX
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
unset CONDA_SHLVL
export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

# Source ROS2 workspace
source install/setup.bash

# Run object tracker with MoveIt integration
/usr/bin/python3 object_to_base_node.py --ros-args -p use_sim_time:=true "$@"
