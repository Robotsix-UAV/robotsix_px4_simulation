# robotsix_px4_simulation
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A ROS2 package for PX4 SITL simulation with Gazebo. This package provides an action server to 
start and stop PX4-based simulations, and automatically manages the lifecycle of all related 
processes.

## Overview

The `robotsix_px4_simulation` package provides the following capabilities:

- **Automatic PX4 Installation**: Clones and builds PX4-Autopilot and MicroXRCE-DDS Agent 
  automatically during package build
- **Action-Based Interface**: ROS2 action server interface for programmatic control of simulations
- **Pre-Configured Models**: Ready-to-use quadcopter models with GPS or mocap sensors
- **Process Management**: Proper handling of all simulation processes (Gazebo, PX4, 
  MicroXRCE-DDS Agent)

This package works in tandem with the `robotsix_px4_sim_interface` package, which defines the 
action interfaces and provides command-line tools.

## Compatibility

For PX4 v1.14.x, the package is compatible with Ubuntu 22.04, ROS2 Humble, and Gazebo Garden.

- PX4 v1.14 SITL compilation fails on Ubuntu 24.04 with default installation.
- PX4 v1.14 SITL fails to detect Gazebo Harmonic (gz-bridge is not compiled).

Those compatibility issues may be fixed with a few efforts. Contributions are welcome!

## Installation

### 1. Clone the repositories

```bash
cd ~/ros2_ws/src
git clone https://github.com/robotsix-UAV/robotsix_px4_sim_interface.git
git clone https://github.com/robotsix-UAV/robotsix_px4_simulation.git
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install -y --from-paths src --ignore-src
```

### 3. Build the packages

```bash
cd ~/ros2_ws
colcon build --packages-select robotsix_px4_sim_interface robotsix_px4_simulation
```

Since the package automatically clones and builds the PX4-Autopilot repository, it may take a while.

You can select a specific PX4 Autopilot version by setting the `PX4_VERSION` parameter:

```bash
# Build with PX4 v1.14.3
colcon build --packages-select robotsix_px4_simulation --cmake-args -DPX4_VERSION=v1.14.3
```

Currently supported PX4 versions: v1.14.0, v1.14.1, v1.14.2, v1.14.3, v1.14.4

### 4. Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Starting the Simulation Server

The simulation server provides the ROS2 action server interface for controlling simulations:

```bash
# Launch the simulation server
ros2 launch robotsix_px4_simulation simulation_server.launch.py
```

#### Headless Mode

To run the simulation without the Gazebo GUI (useful for servers or CI systems), use:

```bash
# Launch in headless mode (no GUI)
ros2 launch robotsix_px4_simulation simulation_server.launch.py headless_mode:=true
```

#### Debugging Simulations

By default, the simulation server hides the output from child processes (PX4, Gazebo, etc.) to keep
the console clean. If you're experiencing issues with simulation launching or running, you can enable
subprocess output for debugging:

```bash
# Launch with subprocess output visible for debugging
ros2 launch robotsix_px4_simulation simulation_server.launch.py hide_simulation_process_output:=false
```

This will show all output from PX4, Gazebo, and other child processes, which can be helpful for
diagnosing simulation problems.

#### Custom PX4 Parameters

To provide a custom PX4 parameter file, you can either add it to the `px4_parameters` folder in the
package and build the package, or specify a custom folder containing your parameter files when
launching the simulation server:

```bash
# Launch with custom PX4 parameters folder
ros2 launch robotsix_px4_simulation simulation_server.launch.py px4_params_folder:=/path/to/your/param_files
```

Files in the specified folder must follow the PX4 airframe naming convention: a numeric ID followed
by an underscore and descriptive name (e.g., `6661_quad_gps`, `6662_quad_mocap`). The numeric prefix
identifies the airframe type and is required by PX4. These files will be copied to the PX4 airframes
directory at launch time.

### Using the Simulation Interface

Once the server is running, you can use ros2 action commands to start and stop simulations. 
Please refer to robotsix_px4_sim_interface/README for more details on the action interfaces.

### Models and Worlds

Any model or world in the 'GZ_SIM_RESOURCE_PATH' can be used in the simulation. The server supports
parsing of xacro model files (as in the provided models).

The package includes pre-configured generic drone models:

- **quad_gps**: Quadcopter with GPS for outdoor use
- **quad_mocap**: Quadcopter with mocap markers for indoor use

The models and worlds provided in this package are included in the GZ_SIM_RESOURCE_PATH when the
package is sourced. To provide your own models or worlds, you can either:
- Place them in the `robotsix_px4_simulation/models` or `robotsix_px4_simulation/worlds` directories
- Add the path to your models or worlds to the `GZ_SIM_RESOURCE_PATH` environment variable

Note: If you provide a custom model, you would probably also need to provide a custom PX4 parameter
file. The package does not automatically generate PX4 parameters for custom models.

To use a model in the simulation, you can specify its directory and name in the action goal. For
example, to start a simulation with the `quad_gps` model:

```bash
ros2 action send_goal /start_simulation robotsix_px4_sim_interface/action/StartSimulation \
  "{world: 'robotsix_px4_simulation/worlds/default.sdf', \
  models: [{model_dir: 'robotsix_px4_simulation/models/quad_gps', \
  model_name: 'uav0', px4_parameters: 6661, x: 0.0, y: 0.0, z: 0.0}]}"
```

PX4 models can also be used, but there is no need to specify the package name:

```bash
ros2 action send_goal /start_simulation robotsix_px4_sim_interface/action/StartSimulation \
  "{world: 'robotsix_px4_simulation/worlds/default.sdf', \
  models: [{model_dir: 'x500', \
  model_name: 'x500', px4_parameters: 4001, x: 0.0, y: 0.0, z: 0.0}]}"
```

### DDS Topics

To configure which PX4 topics are published over DDS, edit the configuration file 
config/dds_topics/default.yaml

## Docker

A Docker setup is provided for easy deployment. The Docker image includes:
- ROS2 Humble
- Gazebo Garden
- Pre-built ROS2 workspace with robotsix_px4_simulation and robotsix_px4_sim_interface packages
- Support for custom models, worlds, and PX4 parameters through volume mounts

### Running the Docker container

```bash
# Basic usage - starts the simulation server in headless mode
docker run --network=host robotsix/px4_simulation

# Using with custom Gazebo resources (models, worlds) and parameters
docker run --network=host \
  -v /path/to/your/gz_resources:/custom_gz_resources \
  -v /path/to/your/parameters:/custom_px4_params \
  robotsix/px4_simulation
```

#### Configuring Simulation Options

You can configure simulation options through environment variables:

```bash
# Run with GUI (not headless) and show subprocess output
docker run --network=host \
  -e HEADLESS_MODE=false \
  -e HIDE_OUTPUT=false \
  robotsix/px4_simulation
```

Available environment variables:
- `HEADLESS_MODE`: Set to `false` to run with Gazebo GUI (default: `true`)
- `HIDE_OUTPUT`: Set to `false` to show simulation process output (default: `true`)

### Building the Docker image

```bash
# From the docker directory
docker build -t robotsix/px4_simulation .
```

## TODO
- Add support for PX4 v1.15.x
- Parameter generation
  - Automatic generation of PX4 parameters for quadcopters
  - Match parameters with those used by xacro files
- Simulation speed control
  - Add parameters for faster/slower than real-time simulation
  - Ensure stability at different simulation speeds
- Fix compatibility issues for v1.14.x
  - Fix compatibility issues with Ubuntu 24.04
  - Fix compatibility issues with Gazebo Harmonic
- Docker option to verbose the subprocess output
- Check of existing PX4 parameter numbers to avoid conflicts
- Add the keyword WORLD in the model directory to allow spawning models from the .sdf world file instead of
  the model directory
