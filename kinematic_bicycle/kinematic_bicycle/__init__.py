from .bicycle_model import *

"""
# Clean build
cd ~/control_ws
rm -rf build install log

# Build
colcon build --packages-select kinematic_bicycle

# Source
source install/setup.bash

# Launch everything
ros2 launch kinematic_bicycle kinematic_bicycle.launch.py
"""