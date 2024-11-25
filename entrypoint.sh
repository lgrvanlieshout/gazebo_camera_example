#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Enable error signals
set -e
 
# Source ROS 2
source /opt/ros/humble/setup.bash

if [ -d ./simulation ]
then
  cd simulation
fi

# Build the workspace if not built
if ! [ -f ./install/setup.bash ]
then
  colcon build --symlink-install
fi

# Source the overlay workspace
source install/setup.bash
 
# Execute the command passed into this entrypoint
echo "Provided arguments: $@"
exec $@
