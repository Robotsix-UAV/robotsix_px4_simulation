# robotsix_px4_simulation
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A ROS2 package for PX4 SITL simulation with Gazebo. This package provides an action server to 
start and stop PX4-based simulations, and automatically manages the lifecycle of all related 
processes.

## Overview

The `robotsix_px4_simulation` package provides the following capabilities:

- **Action-Based Interface**: ROS2 action server interface for programmatic control of simulations
- **Pre-Configured Models**: Ready-to-use quadcopter models with GPS or mocap sensors (defined in xacro format)
- **Process Management**: Proper handling of all simulation processes (Gazebo, PX4,
  MicroXRCE-DDS Agent)
- **Flexible Deployment**: Docker support for easy deployment with pre-built PX4 and MicroXRCE-DDS Agent

This package works in tandem with the `robotsix_px4_sim_interface` package, which defines the 
action interfaces and provides command-line tools.

## Compatibility

This package supports PX4 v1.15.x and v1.16.0 (v1.15.1, v1.15.2, v1.15.3, v1.15.4, v1.16.0) and is compatible with:
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

> **Note:** For PX4 v1.14.x support, please use the v1.14 branch.

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

> **Note:** This package does NOT automatically install PX4-Autopilot or MicroXRCE-DDS Agent. You must provide paths to existing installations when launching the simulation server, or use the provided Docker images which include pre-built PX4 and MicroXRCE-DDS Agent.

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

By default, the simulation server shows the output from child processes (PX4, Gazebo, etc.). If you want to hide this output to keep the console clean, you can disable it:

```bash
# Launch with subprocess output hidden
ros2 launch robotsix_px4_simulation simulation_server.launch.py hide_simulation_process_output:=true
```

#### Custom PX4 Parameters

To provide custom PX4 parameter files, add them to the `config/px4_parameters` folder in the
package and rebuild. The files will be automatically copied to the PX4 airframes directory at launch time.

Files must follow the PX4 airframe naming convention: a numeric ID followed
by an underscore and descriptive name (e.g., `6661_quad_gps`, `6662_quad_mocap`). The numeric prefix
identifies the airframe type and is required by PX4.

The package includes two pre-configured parameter files:
- `6661_quad_gps`: Quadcopter with GPS configuration
- `6662_quad_mocap`: Quadcopter with mocap configuration

#### Custom PX4 and MicroXRCE-DDS Agent Executables

You must provide paths to your PX4 and MicroXRCE-DDS Agent installations when launching the simulation server:

- `px4_dir`: Path to PX4 build directory (should point to the `px4_sitl_default` directory or equivalent)
- `xrce_agent_path`: Path to MicroXRCE-DDS Agent executable

```bash
# Launch with custom PX4 build directory
ros2 launch robotsix_px4_simulation simulation_server.launch.py px4_dir:=/path/to/your/px4_sitl_default

# Launch with custom MicroXRCE-DDS Agent executable
ros2 launch robotsix_px4_simulation simulation_server.launch.py xrce_agent_path:=/path/to/your/MicroXRCEAgent

# Launch with both custom paths
ros2 launch robotsix_px4_simulation simulation_server.launch.py \
  px4_dir:=/path/to/your/px4_sitl_default \
  xrce_agent_path:=/path/to/your/MicroXRCEAgent
```

The default paths are:
- `px4_dir`: `/root/PX4-Autopilot/build/px4_sitl_default/`
- `xrce_agent_path`: `/root/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`

These defaults match the Docker base image configuration.

### Using the Simulation Interface

Once the server is running, you can use ros2 action commands to start and stop simulations. 
Please refer to robotsix_px4_sim_interface/README for more details on the action interfaces.

### Models and Worlds

Any model or world in the `GZ_SIM_RESOURCE_PATH` can be used in the simulation. The server supports
xacro model files which are processed during package installation.

The package includes pre-configured generic drone models (defined as xacro files):

- **quad_gps**: Quadcopter with GPS for outdoor use
- **quad_mocap**: Quadcopter with mocap markers for indoor use

The models and worlds provided in this package are automatically included in the `GZ_SIM_RESOURCE_PATH` when the
package is sourced. To provide your own models or worlds, you can either:
- Place them in the source `models/` or `worlds/` directories and rebuild the package
- Add the path to your models or worlds to the `GZ_SIM_RESOURCE_PATH` environment variable

> **Note:** Model files in `.xacro` format are automatically processed to `.sdf` during package installation. If you provide a custom model, you'll also need to provide a corresponding PX4 parameter file.

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

To configure which PX4 topics are published over DDS:

1. Create a DDS configuration YAML file (see PX4 documentation for format)
2. For Docker deployments: Place it at `docker/dds_topics.yaml` (this file is copied to the base image during Docker build)
3. For local deployments: Configure PX4's DDS settings according to PX4 documentation

> **Note:** The package source does not include a default DDS topics configuration file in `config/dds_topics/`. The Docker setup uses `docker/dds_topics.yaml`.

## Docker

A two-stage Docker setup is provided for flexible deployment:

1. **Base Image** (`Dockerfile.base`): Contains ROS2, Gazebo, PX4 Autopilot, and MicroXRCE-DDS Agent
2. **Deployment Image** (`Dockerfile`): Adds the robotsix_px4_simulation and robotsix_px4_sim_interface packages

### Prerequisites

Before building the Docker images, ensure you have a DDS configuration file:

```bash
# Create or customize docker/dds_topics.yaml with your DDS topic configuration
# See PX4 documentation for the YAML format
```

The `docker/dds_topics.yaml` file should already exist in the repository. You can customize it before building to configure which PX4 topics are published over DDS.

### Building the Base Image

The base image includes:
- ROS2 Jazzy and Gazebo Harmonic
- PX4 Autopilot (with Python virtual environment and custom DDS configuration)
- MicroXRCE-DDS Agent (built with Release mode and system FastDDS/FastCDR)

**Build Arguments:**
- `PX4_REPO`: PX4 repository URL (default: `https://github.com/PX4/PX4-Autopilot.git`)
- `PX4_VERSION`: PX4 version/tag (default: `v1.16.0`)
- `MICRO_XRCE_DDS_VERSION`: MicroXRCE-DDS Agent version/tag (default: `v2.4.3`)

**Build examples:**

```bash
# Default build (PX4 v1.16.0, MicroXRCE-DDS v2.4.3)
cd docker
docker build -f Dockerfile.base -t robotsix/px4-sim-base:latest .

# Custom PX4 repository and versions
cd docker
docker build -f Dockerfile.base \
  --build-arg PX4_REPO=https://github.com/your-org/PX4-Autopilot.git \
  --build-arg PX4_VERSION=v1.15.4 \
  --build-arg MICRO_XRCE_DDS_VERSION=v2.4.2 \
  -t robotsix/px4-sim-base:custom .
```

### Building the Deployment Image

The deployment image adds the ROS2 packages and uses PX4/MicroXRCE-DDS from the base image.

**Build Arguments:**
- `BASE_IMAGE`: Base image to use (default: `robotsix/px4-sim-base:latest`)
- `GIT_REF`: Git branch/tag for robotsix_px4_simulation (default: `main`)

**Build examples:**

```bash
# Using default base image
cd docker
docker build -f Dockerfile -t robotsix/px4-sim:latest .

# Using custom base image
cd docker
docker build -f Dockerfile \
  --build-arg BASE_IMAGE=robotsix/px4-sim-base:custom \
  --build-arg GIT_REF=develop \
  -t robotsix/px4-sim:develop .
```

### Complete Build Example

```bash
# Step 1: Copy DDS configuration (if not already done)
cp config/dds_topics/default.yaml docker/dds_topics.yaml

# Step 2: Build base image with specific versions
cd docker
docker build -f Dockerfile.base \
  --build-arg PX4_VERSION=v1.15.4 \
  --build-arg MICRO_XRCE_DDS_VERSION=v2.4.3 \
  -t robotsix/px4-sim-base:v1.15.4 .

# Step 3: Build deployment image
docker build -f Dockerfile \
  --build-arg BASE_IMAGE=robotsix/px4-sim-base:v1.15.4 \
  -t robotsix/px4-sim:latest .
```

### Running the Docker Container

```bash
# Basic usage - headless mode
docker run --network=host robotsix/px4-sim:latest

# With GUI (requires X11 permissions)
docker run --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e HEADLESS_MODE=false \
  robotsix/px4-sim:latest

# With custom models
docker run --network=host \
  -v /path/to/custom/models:/custom_gz_resources \
  robotsix/px4-sim:latest

# Override PX4 or MicroXRCE-DDS paths (advanced)
docker run --network=host \
  -e PX4_DIR=/custom/px4/build/path \
  -e XRCE_AGENT_PATH=/custom/agent/path \
  robotsix/px4-sim:latest
```

**Available Environment Variables:**
- `HEADLESS_MODE`: Set to `false` to run with Gazebo GUI (default: `true`)
- `HIDE_OUTPUT`: Set to `false` to show simulation process output (default: `true`)
- `PX4_DIR`: Path to PX4 build directory (default: `/root/PX4-Autopilot/build/px4_sitl_default`)
- `XRCE_AGENT_PATH`: Path to MicroXRCEAgent executable (default: `/root/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`)

## TODO
- Parameter generation
  - Automatic generation of PX4 parameters for quadcopters
  - Match parameters with those used by xacro files
- Simulation speed control
  - Add parameters for faster/slower than real-time simulation
  - Ensure stability at different simulation speeds
- Add the keyword WORLD in the model directory to allow spawning models from the .sdf world file instead of
  the model directory
