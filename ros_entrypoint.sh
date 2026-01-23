#!/bin/bash
set -e
# Source the base ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source the workspace if it exists
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# Execute the command given to the container
exec "$@"
