#!/bin/bash
# Build script for the robot project

echo "========================================"
echo "Building Robot Ball Chaser Project"
echo "========================================"

# Source ROS 2
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /home/mehmetAkifVardar/robot

# Clean previous build (optional)
# echo "Cleaning previous build..."
# rm -rf build install log

# Build with symlink install
echo "Building packages..."
colcon build --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "Build successful!"
    echo "========================================"
    echo ""
    echo "To run the project:"
    echo "  1. Source the workspace:"
    echo "     source install/setup.bash"
    echo ""
    echo "  2. Launch the system:"
    echo "     ros2 launch my_robot_bringup bringup.launch.py"
    echo ""
else
    echo ""
    echo "========================================"
    echo "Build failed! Check errors above."
    echo "========================================"
    exit 1
fi
