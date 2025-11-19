#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash

# Add custom Gazebo resources (models, worlds, etc.) to Gazebo resource path if the directory is not empty
if [ -d "/custom_gz_resources" ] && [ "$(ls -A /custom_gz_resources)" ]; then
  echo "Custom Gazebo resources directory found. Adding to GZ_SIM_RESOURCE_PATH."
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/custom_gz_resources
fi

# Start the simulation server with provided arguments
if [ "$1" = "sim" ]; then
  # Set default values if environment variables are not provided
  HEADLESS_MODE=${HEADLESS_MODE:-true}
  HIDE_OUTPUT=${HIDE_OUTPUT:-true}
  
  # Set paths to PX4 and MicroXRCE-DDS Agent from base image
  PX4_DIR=${PX4_DIR:-/root/PX4-Autopilot/build/px4_sitl_default}
  XRCE_AGENT_PATH=${XRCE_AGENT_PATH:-/root/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}
  
  echo "Starting simulation server..."
  echo "- Headless mode: ${HEADLESS_MODE}"
  echo "- Hide process output: ${HIDE_OUTPUT}"
  echo "- PX4 directory: ${PX4_DIR}"
  echo "- MicroXRCE-DDS Agent path: ${XRCE_AGENT_PATH}"
  
  exec ros2 launch robotsix_px4_simulation simulation_server.launch.py \
    headless_mode:=${HEADLESS_MODE} \
    px4_params_folder:=/custom_px4_params \
    hide_simulation_process_output:=${HIDE_OUTPUT} \
    px4_dir:=${PX4_DIR} \
    xrce_agent_path:=${XRCE_AGENT_PATH}
else
  # If the first argument is not "sim", pass all arguments to bash
  exec "$@"
fi
