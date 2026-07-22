#!/bin/bash
set -e

# Source ROS
source /opt/ros/jazzy/setup.bash

if [ -f /home/user/workspace/install/setup.bash ]; then
	source /home/user/workspace/install/setup.bash
fi

# Hand off to the requested command (default: bash)
exec "$@"
