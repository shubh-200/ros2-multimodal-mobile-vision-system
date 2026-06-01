#!/bin/bash
set -e

# Source the ROS 2 core
source /opt/ros/jazzy/setup.bash

# Source the custom workspace
source /ros2_ws/install/setup.bash

# Execute the command passed into the docker run/compose command
exec "$@"