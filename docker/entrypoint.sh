#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash

# All gz-transport participants (gz sim server/GUI, ros_gz_bridge, ros_gz_sim
# create, PX4) run inside this container, so discovery only needs loopback.
# Left unset, gz-transport auto-selects the first non-loopback interface; on
# hosts where that interface does not loop multicast back (Wi-Fi is a common
# case, and --network=host exposes it), discovery silently fails and the spawn
# hangs on "Requesting list of world names". Override to reach gz-transport
# peers outside the container.
export GZ_IP=${GZ_IP:-127.0.0.1}

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
  echo "- gz-transport IP: ${GZ_IP}"
  
  exec ros2 launch robotsix_px4_simulation simulation_server.launch.py \
    headless_mode:=${HEADLESS_MODE} \
    hide_simulation_process_output:=${HIDE_OUTPUT} \
    px4_dir:=${PX4_DIR} \
    xrce_agent_path:=${XRCE_AGENT_PATH}
else
  # If the first argument is not "sim", pass all arguments to bash
  exec "$@"
fi
