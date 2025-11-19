# SPDX-License-Identifier: Apache-2.0

# Author: Damien SIX (damien@robotsix.net)
# Copyright: Robotsix
# Date: 2025

import os
import glob
import shutil
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def copy_px4_params(context):
    """Copy PX4 parameter files from specified folder to PX4 airframes directory."""
    # Get paths
    pkg_dir = get_package_share_directory("robotsix_px4_simulation")
    params_folder = os.path.join(pkg_dir, "config", "px4_parameters")
    
    # Get px4_dir from launch configuration
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    airframes_dir = os.path.join(px4_dir, "etc", "init.d-posix", "airframes")

    if params_folder and os.path.isdir(params_folder):
        # Ensure the target directory exists
        os.makedirs(airframes_dir, exist_ok=True)

        # Find all files in the specified folder
        param_files = glob.glob(os.path.join(params_folder, "*"))
        for file_path in param_files:
            file_name = os.path.basename(file_path)
            # Check if the filename matches the pattern {digits}_*
            if re.match(r"^\d+_", file_name):
                # Copy the file to the PX4 airframes directory
                shutil.copy(file_path, os.path.join(airframes_dir, file_name))
                print(f"Copied PX4 parameter file: {file_name}")

    # Create simulation server node (Python implementation)
    simulation_server = Node(
        package="robotsix_px4_simulation",
        executable="simulation_server",
        name="simulation_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "hide_simulation_process_output": LaunchConfiguration(
                    "hide_simulation_process_output"
                ),
                "headless_mode": LaunchConfiguration("headless_mode"),
                "px4_dir": LaunchConfiguration("px4_dir"),
                "xrce_agent_path": LaunchConfiguration("xrce_agent_path"),
            }
        ],
    )

    return [simulation_server]


def generate_launch_description():
    # Declare launch argument for hiding subprocess output
    hide_output_arg = DeclareLaunchArgument(
        "hide_simulation_process_output",
        default_value="false",
        description="Hide output from simulation processes (PX4, Gazebo, etc.)",
    )
    
    # Declare launch argument for headless mode
    headless_arg = DeclareLaunchArgument(
        "headless_mode",
        default_value="true",
        description="Run Gazebo in headless mode without GUI",
    )
    
    # Declare launch argument for PX4 build directory
    px4_dir_arg = DeclareLaunchArgument(
        "px4_dir",
        default_value="/root/PX4-Autopilot/build/px4_sitl_default/",
        description="Path to PX4 build directory (containing bin/px4 and etc/)",
    )
    
    # Declare launch argument for MicroXRCE-DDS Agent path
    xrce_agent_path_arg = DeclareLaunchArgument(
        "xrce_agent_path",
        default_value="/root/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent",
        description="Path to MicroXRCE-DDS Agent executable",
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the arguments
    ld.add_action(hide_output_arg)
    ld.add_action(headless_arg)
    ld.add_action(px4_dir_arg)
    ld.add_action(xrce_agent_path_arg)

    # Add the setup and node launch via OpaqueFunction
    # This ensures the parameter copying happens before the node is created
    ld.add_action(OpaqueFunction(function=copy_px4_params))

    return ld
