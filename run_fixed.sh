#!/bin/bash
# Run with proper Qt/OpenGL settings for NVIDIA

echo "========================================"
echo "Launching with proper Qt/GL settings"
echo "========================================"

# Kill any existing processes
pkill -9 -f "gz|rviz" 2>/dev/null

# NVIDIA settings
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Qt/OpenGL settings for X11
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=0

# Gazebo settings
export GZ_VERSION=harmonic
export GAZEBO_MODEL_PATH=/usr/share/gazebo/models:$GAZEBO_MODEL_PATH
export OGRE_RTshader_WRITE_SHADERS_TO_DISK=1

# Source 
source /opt/ros/humble/setup.bash
cd /home/mehmetAkifVardar/robot
source install/setup.bash

ros2 launch my_robot_bringup bringup_staggered.launch.py
