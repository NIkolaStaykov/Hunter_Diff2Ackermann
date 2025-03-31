#!/bin/bash
# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /workspaces/Sevensense/ackermann_bridge_ws/install/setup.bash

# Launch node
ros2 launch ackermann_bridge bridge.launch.py
