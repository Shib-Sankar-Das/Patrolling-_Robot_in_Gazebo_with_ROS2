#!/bin/bash
# ============================================================================
# PATROL ROBOT SETUP SCRIPT
# ============================================================================
# This script sets up the environment for running the patrol robot project
# It switches DDS from FastDDS (problematic) to CycloneDDS (reliable)
# ============================================================================

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace
source /home/shib/ros2_ws/install/setup.bash

# ============================================================================
# DDS CONFIGURATION - THE CRITICAL FIX
# ============================================================================
# Switch from FastDDS to CycloneDDS to fix:
# - "Discarding message because queue is full" warnings
# - Nav2 communication failures
# - Initial pose not being received
# - Lifecycle manager not responding
# ============================================================================

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Point to our custom CycloneDDS configuration with larger buffers
export CYCLONEDDS_URI=file:///home/shib/ros2_ws/src/two_wheeled_robot/config/cyclonedds.xml

# ============================================================================
# GAZEBO CONFIGURATION
# ============================================================================
# Set Gazebo model path to find our custom robot and world models
export GAZEBO_MODEL_PATH=/home/shib/ros2_ws/src/two_wheeled_robot/models:$GAZEBO_MODEL_PATH

# Reduce Gazebo verbosity (optional - reduces terminal clutter)
export GAZEBO_VERBOSE=0

# ============================================================================
# ROS 2 DOMAIN ID (Optional - useful if multiple robots on same network)
# ============================================================================
export ROS_DOMAIN_ID=0

# ============================================================================
# VERIFICATION
# ============================================================================
echo "=============================================="
echo "PATROL ROBOT ENVIRONMENT SETUP COMPLETE"
echo "=============================================="
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "CycloneDDS Config: $CYCLONEDDS_URI"
echo "Gazebo Model Path: $GAZEBO_MODEL_PATH"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo "=============================================="
echo ""
echo "To run the simulation, execute:"
echo "  ros2 launch two_wheeled_robot house_world_v1.launch.py"
echo ""
echo "Then in a NEW terminal (after sourcing this script again):"
echo "  ros2 run two_wheeled_robot my_patrol_robot.py"
echo "=============================================="
