#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Add custom Gazebo resources (models, worlds, etc.) to Gazebo resource path if the directory is not empty
if [ -d "/custom_gz_resources" ] && [ "$(ls -A /custom_gz_resources)" ]; then
  echo "Custom Gazebo resources directory found. Adding to GZ_SIM_RESOURCE_PATH."
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/custom_gz_resources
fi

# Start the simulation server with provided arguments
if [ "$1" = "sim" ]; then
  echo "Starting simulation server in headless mode..."
  exec ros2 launch robotsix_px4_simulation simulation_server.launch.py headless_mode:=true px4_params_folder:=/custom_px4_params hide_simulation_process_output:=true
else
  # If the first argument is not "sim", pass all arguments to bash
  exec "$@"
fi
