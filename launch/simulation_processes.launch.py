#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""
Complete simulation launch file
Handles Gazebo, model spawning, PX4 instances, and MicroXRCE-DDS Agent
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, Shutdown)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import json
from tempfile import NamedTemporaryFile
import xacro


def process_model_dir(model_dir):
    """
    Process model directory to generate SDF if needed.
    If model.sdf.xacro exists, generate model.sdf in the same directory.
    Returns the model directory path for Gazebo.
    """
    xacro_file = os.path.join(model_dir, 'model.sdf.xacro')
    sdf_file = os.path.join(model_dir, 'model.sdf')
    
    # If xacro file exists, process it to generate model.sdf
    if os.path.exists(xacro_file):
        doc = xacro.process_file(xacro_file)
        xml = doc.toprettyxml(indent='  ')
        with open(sdf_file, 'w') as f:
            f.write(xml)
        print(f"Generated SDF from Xacro: {sdf_file}")
    
    # Return the model directory for Gazebo
    return


def launch_simulation(context):
    """Launch all simulation components"""
    
    # Get launch configurations
    world = context.launch_configurations.get('world', '')
    headless_mode = context.launch_configurations.get(
        'headless_mode', 'false')
    hide_output = context.launch_configurations.get(
        'hide_simulation_process_output', 'true')
    px4_dir = context.launch_configurations.get('px4_dir', '')
    xrce_agent_path = context.launch_configurations.get(
        'xrce_agent_path', '')
    models_json = context.launch_configurations.get('models', '[]')
    
    # Parse models
    try:
        models = json.loads(models_json)
    except json.JSONDecodeError:
        models = []
    
    # Get package paths
    pkg_share = get_package_share_directory('robotsix_px4_simulation')
    
    # Set defaults
    if not px4_dir:
        px4_dir = os.path.join(pkg_share, 'px4_sitl_default')
    if not xrce_agent_path:
        xrce_agent_path = os.path.join(pkg_share, 'bin', 'MicroXRCEAgent')
    
    output_mode = 'log' if hide_output == 'true' else 'screen'
    
    processes = []
    
    # Prepare Gazebo environment variables
    gz_env = dict(os.environ)
    
    # Add GZ_SIM_RESOURCE_PATH for models and resources
    install_prefix = os.path.dirname(os.path.dirname(pkg_share))
    gz_resource_paths = [
        os.path.join(install_prefix, 'share'),
        os.path.join(pkg_share, 'px4_models')
    ]
    
    # Prepend to existing GZ_SIM_RESOURCE_PATH if it exists
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_paths.append(existing_gz_path)
    gz_env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(gz_resource_paths)
    
    # Add SDF_PATH
    sdf_paths = [os.path.join(install_prefix, 'share')]
    existing_sdf_path = os.environ.get('SDF_PATH', '')
    if existing_sdf_path:
        sdf_paths.append(existing_sdf_path)
    gz_env['SDF_PATH'] = os.pathsep.join(sdf_paths)
    
    # 1. Start Gazebo
    gz_cmd = ['gz', 'sim', '-r', world]
    if headless_mode == 'true':
        gz_cmd.insert(2, '-s')  # Insert headless flag
    
    gazebo_process = ExecuteProcess(
        cmd=gz_cmd,
        output=output_mode,
        name='gazebo',
        env=gz_env,
        on_exit=Shutdown()
    )
    processes.append(gazebo_process)

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output="screen",
    )

    processes.append(gz_bridge)
    
    # 2. Spawn models
    
    for idx, model in enumerate(models):
        # Process model directory (generate SDF from xacro if needed)
        model_dir = model['model_dir']
        process_model_dir(model_dir)
        
        # Position offset for each model
        x = model.get('x', 0.0)
        y = model.get('y', 0.0)
        z = model.get('z', 0.1)
        
        # Build spawn command using ros_gz_sim create
        spawn_cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', model_dir,
            '-name', model['model_name'],
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ]
        
        # Spawn each model
        spawn_process = ExecuteProcess(
                    cmd=spawn_cmd,
                    output=output_mode,
                    name=f'spawn_{model["model_name"]}'
                )

        processes.append(spawn_process)
    
    # 3. Start PX4 instances for each model
    px4_exec = os.path.join(px4_dir, 'bin', 'px4')
    
    for idx, model in enumerate(models):
        # PX4 command
        px4_cmd = [
            px4_exec,
            os.path.join(px4_dir, 'etc'),
            '-s',
            os.path.join(px4_dir, 'etc', 'init.d-posix', 'rcS'),
            '-w',
            px4_dir,
            '-i',
            str(idx)
        ]
        
        # Environment variables for PX4
        px4_env = {
            'PX4_SYS_AUTOSTART': str(model['px4_parameters']),
            'PX4_GZ_MODEL_NAME': model['model_name'],
            'PX4_UXRCE_DDS_NS': model['model_name']
        }
        
        # Merge with existing environment
        full_env = dict(os.environ)
        full_env.update(px4_env)
        
        px4_process = ExecuteProcess(
            cmd=px4_cmd,
            output=output_mode,
            name=f'px4_{model["model_name"]}',
            env=full_env,
            on_exit=Shutdown()
        )
        
        processes.append(px4_process)
    
    # 4. Start MicroXRCE-DDS Agent
    xrce_cmd = [xrce_agent_path, 'udp4', '-p', '8888']
    
    xrce_process = ExecuteProcess(
        cmd=xrce_cmd,
        output=output_mode,
        name='micro_xrce_agent',
        on_exit=Shutdown()
    )
    
    processes.append(xrce_process)
    
    return processes


def generate_launch_description():
    """Generate launch description with all arguments"""
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world file'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless_mode',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    hide_output_arg = DeclareLaunchArgument(
        'hide_simulation_process_output',
        default_value='true',
        description='Hide simulation process output'
    )
    
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value='',
        description='Path to PX4 build directory'
    )
    
    xrce_agent_path_arg = DeclareLaunchArgument(
        'xrce_agent_path',
        default_value='',
        description='Path to MicroXRCEAgent executable'
    )
    
    models_arg = DeclareLaunchArgument(
        'models',
        default_value='[]',
        description='JSON string of models to spawn'
    )
    
    return LaunchDescription([
        world_arg,
        headless_arg,
        hide_output_arg,
        px4_dir_arg,
        xrce_agent_path_arg,
        models_arg,
        OpaqueFunction(function=launch_simulation)
    ])