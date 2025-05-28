// SPDX-License-Identifier: Apache-2.0

/**
 * @file
 * @brief Server node for managing Gazebo simulations with PX4 flight
 * controllers
 * @author Damien SIX
 * @date 2025
 * @copyright Robotsix
 */

#pragma once

#include <atomic>
#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotsix_px4_sim_interface/action/start_simulation.hpp>
#include <robotsix_px4_sim_interface/action/stop_simulation.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

/**
 * @brief Server node for managing Gazebo simulations with PX4 flight
 * controllers
 *
 * This class provides ROS 2 action servers to start and stop Gazebo simulations
 * with PX4 flight controllers. It handles:
 * - Launching Gazebo simulation process
 * - Spawning vehicle models in the simulation
 * - Starting PX4 SITL instances for each model
 * - Setting up communication via MicroXRCEAgent
 * - Proper cleanup and termination of all processes
 */

class SimulationServer : public rclcpp::Node {
public:
  /** @brief Type alias for StartSimulation action */
  using StartSim = robotsix_px4_sim_interface::action::StartSimulation;
  /** @brief Type alias for StopSimulation action */
  using StopSim = robotsix_px4_sim_interface::action::StopSimulation;
  /** @brief Type alias for StartSimulation goal handle */
  using GoalHandleStartSim = rclcpp_action::ServerGoalHandle<StartSim>;
  /** @brief Type alias for StopSimulation goal handle */
  using GoalHandleStopSim = rclcpp_action::ServerGoalHandle<StopSim>;

  /**
   * @brief Constructor for SimulationServer
   *
   * Initializes the node, declares parameters, and sets up the action servers
   * for starting and stopping simulations.
   */
  SimulationServer();

  /**
   * @brief Flag to control if simulation process output is shown
   *
   * When true, the output from Gazebo, PX4, and other processes is
   * redirected to /dev/null
   */
  bool hide_simulation_process_output_;

  /** @brief Flag to control if Gazebo runs in headless mode without GUI */
  bool headless_mode_;

  /** @brief Path to PX4 executable from parameter */
  std::string px4_path_param_;

  /** @brief Path to MicroXRCE-DDS Agent executable from parameter */
  std::string xrce_agent_path_param_;
  
  /** @brief Validated path to PX4 executable */
  std::string px4_path_;
  
  /** @brief Validated path to MicroXRCE-DDS Agent executable */
  std::string xrce_agent_path_;

private:
  /** @brief Action server for StartSimulation action */
  rclcpp_action::Server<StartSim>::SharedPtr start_action_server_;

  /** @brief Action server for StopSimulation action */
  rclcpp_action::Server<StopSim>::SharedPtr stop_action_server_;

  /** @brief Flag to track if an action is currently being processed */
  std::atomic<bool> action_in_progress_;

  /** @brief Publisher for simulation clock */
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  /** @brief Callback for Gazebo clock messages */
  void on_clock_message(const gz::msgs::Clock &msg);

  /**
   * @brief Handles a new StartSimulation goal request
   *
   * Checks if the goal can be accepted by verifying:
   * - No other action is in progress
   * - Simulation is not already running
   * - Model names in the request are unique
   *
   * @param uuid Goal UUID (unused)
   * @param goal Requested simulation configuration
   * @return GoalResponse ACCEPT_AND_EXECUTE if the goal can be accepted, REJECT
   * otherwise
   */
  rclcpp_action::GoalResponse
  handle_start_goal(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const StartSim::Goal> goal);

  /**
   * @brief Handles cancellation of a StartSimulation goal
   *
   * @param goal_handle Handle to the goal being cancelled
   * @return CancelResponse REJECT as cancellation is not supported
   */
  rclcpp_action::CancelResponse
  handle_start_cancel(const std::shared_ptr<GoalHandleStartSim> goal_handle);

  /**
   * @brief Handles accepted StartSimulation goal
   *
   * Launches a new thread to execute the simulation startup process
   *
   * @param goal_handle Handle to the accepted goal
   */
  void
  handle_start_accepted(const std::shared_ptr<GoalHandleStartSim> goal_handle);

  /**
   * @brief Executes the StartSimulation goal
   *
   * Main implementation of the simulation startup:
   * 1. Launches Gazebo with the requested world
   * 2. Waits for Gazebo to initialize
   * 3. Spawns all requested models
   * 4. Launches PX4 SITL instances for each model
   * 5. Launches MicroXRCEAgent for communication
   *
   * @param goal_handle Handle to the goal being executed
   */
  void execute_start(const std::shared_ptr<GoalHandleStartSim> goal_handle);

  /**
   * @brief Handles a new StopSimulation goal request
   *
   * Checks if the goal can be accepted by verifying:
   * - No other action is in progress
   * - Simulation is currently running
   *
   * @param uuid Goal UUID (unused)
   * @param goal Goal parameters (unused)
   * @return GoalResponse ACCEPT_AND_EXECUTE if the goal can be accepted, REJECT
   * otherwise
   */
  rclcpp_action::GoalResponse
  handle_stop_goal(const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const StopSim::Goal> goal);

  /**
   * @brief Handles cancellation of a StopSimulation goal
   *
   * @param goal_handle Handle to the goal being cancelled
   * @return CancelResponse REJECT as cancellation is not supported
   */
  rclcpp_action::CancelResponse
  handle_stop_cancel(const std::shared_ptr<GoalHandleStopSim> goal_handle);

  /**
   * @brief Handles accepted StopSimulation goal
   *
   * Launches a new thread to execute the simulation shutdown process
   *
   * @param goal_handle Handle to the accepted goal
   */
  void
  handle_stop_accepted(const std::shared_ptr<GoalHandleStopSim> goal_handle);

  /**
   * @brief Executes the StopSimulation goal
   *
   * Terminates all simulation processes in the correct order:
   * 1. PX4 SITL instances
   * 2. MicroXRCEAgent
   * 3. Gazebo
   *
   * @param goal_handle Handle to the goal being executed
   */
  void execute_stop(const std::shared_ptr<GoalHandleStopSim> goal_handle);

  /**
   * @brief Checks if the simulation is currently running
   *
   * Verifies if any of the simulation processes (Gazebo, PX4, XRCEAgent)
   * are still active.
   *
   * @return true if any simulation process is running, false otherwise
   */
  bool is_simulation_running();

  /**
   * @brief Shuts down all simulation processes
   *
   * Performs a clean shutdown of all processes in the correct order:
   * 1. PX4 instances
   * 2. MicroXRCEAgent
   * 3. Gazebo
   *
   * @return true if all processes were terminated successfully, false otherwise
   */
  bool shutdown_simulation();

  /**
   * @brief Launches a new process with the given arguments
   *
   * Forks a child process to execute the specified command, optionally
   * redirecting output to /dev/null based on hide_simulation_process_output_.
   *
   * @param executable_path Path to the executable to launch
   * @param args Vector of command-line arguments
   * @param process_name Descriptive name for logging purposes
   * @return PID of the launched process, or -1 on failure
   */
  pid_t launch_process(const std::string &executable_path,
                       const std::vector<std::string> &args,
                       const std::string &process_name);

  /**
   * @brief Terminates a process with increasing signal severity
   *
   * Attempts to terminate the process with escalating signals:
   * 1. SIGINT
   * 2. SIGTERM
   * 3. SIGKILL
   * With waiting periods between signals to allow graceful shutdown.
   *
   * @param pid Process ID to terminate
   * @param process_name Descriptive name for logging purposes
   * @return true if process was terminated successfully, false otherwise
   */
  bool terminate_process(pid_t pid, const char *process_name);

  /**
   * @brief Waits for Gazebo to be fully initialized
   *
   * Polls for Gazebo world topics to verify that the simulation
   * environment is ready for model spawning.
   *
   * @param timeout_seconds Maximum time to wait in seconds
   * @return true if Gazebo is ready within the timeout, false otherwise
   */
  bool wait_for_gazebo_ready(int timeout_seconds = 30);

  /**
   * @brief Waits for drone ROS2 topics to become available
   *
   * Checks for the presence of /drone_name/fmu/out/timesync_status topics
   * for each drone to confirm PX4 is properly connected to ROS2.
   *
   * @param model_names List of drone model names to check
   * @param timeout_seconds Maximum time to wait in seconds
   * @return true if all drone topics are available within timeout, false
   * otherwise
   */
  bool wait_for_drone_topics(const std::vector<std::string> &model_names,
                             int timeout_seconds = 30);

  /**
   * @brief Spawns a model in the Gazebo simulation
   *
   * Processes the model directory, spawns the model at the specified position,
   * and verifies the model has been successfully loaded in Gazebo.
   *
   * @param model_name Name to assign to the spawned model
   * @param model_dir Directory containing model files
   * @param x X-coordinate for model placement
   * @param y Y-coordinate for model placement
   * @param z Z-coordinate for model placement
   * @param timeout_seconds Maximum time to wait for model to spawn in seconds
   * @return true if model was spawned successfully, false otherwise
   */
  bool spawn_model(const std::string &model_name, const std::string &model_dir,
                   float x, float y, float z, int timeout_seconds = 10);

  /**
   * @brief Processes a model directory for Gazebo compatibility
   *
   * Creates a temporary copy of the model directory and processes any
   * .sdf.xacro files by running the xacro tool to generate plain SDF.
   *
   * @param model_dir Original model directory path
   * @return Path to the processed temporary directory, or empty string on
   * failure
   */
  std::string process_model_dir(const std::string &model_dir);

  /**
   * @brief Finds the absolute path of a model directory
   *
   * Searches for the model directory in the Gazebo resource path
   * (GZ_SIM_RESOURCE_PATH) if the provided path is not absolute.
   *
   * @param model_dir Relative or absolute model directory path
   * @return Absolute path to the model directory, or empty string if not found
   */
  std::string find_model_path(const std::string &model_dir);

  /** @brief Process ID of the Gazebo simulation, -1 if not running */
  pid_t gazebo_pid_ = -1;

  /** @brief Process ID of the MicroXRCEAgent, -1 if not running */
  pid_t xrce_agent_pid_ = -1;

  /** @brief List of process IDs for all PX4 SITL instances */
  std::vector<pid_t> px4_pids_;

  /** @brief Gazebo transport node for communication with the simulation */
  std::shared_ptr<gz::transport::Node> gz_node_;
};
