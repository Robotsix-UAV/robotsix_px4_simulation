# Copyright 2025 Robotsix
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(robotsix_px4_simulation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotsix_px4_sim_interface REQUIRED)

# Try to find gz-transport13 first, fall back to gz-transport12 if not found
find_package(gz-transport13 QUIET)
if(gz-transport13_FOUND)
  set(GZ_TRANSPORT_PKG gz-transport13)
else()
  find_package(gz-transport12 REQUIRED)
  set(GZ_TRANSPORT_PKG gz-transport12)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(ExternalProject)

# Options to skip PX4 and MicroDDS installation
option(SKIP_PX4_INSTALL "Skip PX4 Autopilot download and build" OFF)
option(SKIP_MICRODDS_INSTALL "Skip MicroXRCE-DDS Agent download and build" OFF)

if(NOT SKIP_PX4_INSTALL)
  if(NOT DEFINED PX4_VERSION)
    set(PX4_VERSION "v1.16.0")
  endif()
  set(ALLOWED_PX4_VERSIONS "v1.15.1;v1.15.2;v1.15.3;v1.15.4;v1.16.0")
  list(FIND ALLOWED_PX4_VERSIONS "${PX4_VERSION}" VERSION_INDEX)
  if(VERSION_INDEX EQUAL -1)
    message(FATAL_ERROR "Invalid PX4 version: ${PX4_VERSION}. Allowed versions: ${ALLOWED_PX4_VERSIONS}")
  endif()
  set(PX4_AUTOPILOT_DIR ${CMAKE_BINARY_DIR}/PX4-Autopilot)

  # Setup PX4-Autopilot
  ExternalProject_Add(px4_download
    GIT_REPOSITORY         https://github.com/PX4/PX4-Autopilot.git
    GIT_TAG                ${PX4_VERSION}
    GIT_SHALLOW            ON               
    GIT_SUBMODULES_RECURSE ON              
    GIT_PROGRESS           ON               

    SOURCE_DIR            ${PX4_AUTOPILOT_DIR}

    # Disable all build steps so no targets are generated:
    UPDATE_COMMAND        ""
    CONFIGURE_COMMAND     ""
    BUILD_COMMAND         ""
    INSTALL_COMMAND       ""

    EXCLUDE_FROM_ALL      1
  )

  # Create Python virtual environment for PX4 build
  add_custom_target(px4_venv
    COMMAND python3 -m venv ${PX4_AUTOPILOT_DIR}/px4_venv
    COMMAND ${PX4_AUTOPILOT_DIR}/px4_venv/bin/pip install --upgrade pip
    COMMAND ${PX4_AUTOPILOT_DIR}/px4_venv/bin/pip install --upgrade setuptools wheel twine check-wheel-contents matplotlib numpy pyyaml pyros-genmsg lark kconfiglib future symforce jsonschema
    COMMAND ${PX4_AUTOPILOT_DIR}/px4_venv/bin/pip install empy==3.3.4
    DEPENDS px4_download
    COMMENT "Setting up Python virtual environment for PX4 build"
  )

  # Copy DDS configuration file
  add_custom_target(px4_dds_config
    COMMAND ${CMAKE_COMMAND} -E copy 
      ${CMAKE_CURRENT_SOURCE_DIR}/config/dds_topics/default.yaml
      ${PX4_AUTOPILOT_DIR}/src/modules/uxrce_dds_client/dds_topics.yaml
    DEPENDS px4_venv
    COMMENT "Copying DDS configuration file"
  )

  # Build PX4 SITL
  add_custom_target(px4_autopilot ALL
    COMMAND cd ${PX4_AUTOPILOT_DIR} && . ./px4_venv/bin/activate && make px4_sitl_default
    DEPENDS px4_dds_config
    COMMENT "Building PX4 Autopilot ${PX4_VERSION} SITL"
  )

  # Install PX4 SITL build directory
  install(
    DIRECTORY ${PX4_AUTOPILOT_DIR}/build/px4_sitl_default/
    DESTINATION share/${PROJECT_NAME}/px4_sitl_default
    USE_SOURCE_PERMISSIONS
  )

  # Install PX4 SITL config files to the installed build directory
  file(GLOB CONFIG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/config/px4_parameters/*")
  foreach(CONFIG_FILE ${CONFIG_FILES})
    get_filename_component(CONFIG_NAME ${CONFIG_FILE} NAME)
    install(
      FILES ${CONFIG_FILE}
      DESTINATION share/${PROJECT_NAME}/px4_sitl_default/etc/init.d-posix/airframes
      PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
    )
  endforeach()
  
  # Install PX4 models
  install(
    DIRECTORY ${PX4_AUTOPILOT_DIR}/Tools/simulation/gz/models/
    DESTINATION share/${PROJECT_NAME}/px4_models
  )
endif()

# Setup Micro XRCE-DDS Agent as an external project
if(NOT SKIP_MICRODDS_INSTALL)
  ExternalProject_Add(micro_xrce_dds_agent
    GIT_REPOSITORY https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    GIT_TAG v2.4.3
    GIT_SHALLOW ON
    GIT_PROGRESS ON
    
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DUAGENT_BUILD_EXECUTABLE=ON
      -DUAGENT_USE_SYSTEM_FASTDDS=ON
      -DUAGENT_USE_SYSTEM_FASTCDR=ON

    INSTALL_COMMAND ""
    
    BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/micro_xrce_dds_agent-prefix/src/micro_xrce_dds_agent-build/microxrcedds_agent
  )

  install(
    FILES ${CMAKE_BINARY_DIR}/micro_xrce_dds_agent-prefix/src/micro_xrce_dds_agent-build/MicroXRCEAgent
    DESTINATION share/${PROJECT_NAME}/bin
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
endif()


# Build simulation server
add_executable(simulation_server src/simulation_server.cpp)
ament_target_dependencies(simulation_server
  rclcpp
  rclcpp_action
  robotsix_px4_sim_interface
  ament_index_cpp
  ${GZ_TRANSPORT_PKG}
)
target_include_directories(simulation_server
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS
  simulation_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install config
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

# Install worlds
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.sdf" PATTERN "*.world"
)

# Install models
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)


# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_environment_hooks(
	${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/gz_sim.sh.in
)
ament_package()
