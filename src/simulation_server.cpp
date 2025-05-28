// SPDX-License-Identifier: Apache-2.0

/**
 * @file
 * @brief Implementation of the SimulationServer class
 * @author Damien SIX
 * @date 2025
 * @copyright Robotsix
 */

#include "robotsix_px4_sim_interface/simulation_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/transport/Node.hh>
#include <memory>
#include <rclcpp_action/server.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

// TODO: Check simluation has started with the presence of models, PX4, etc.
// ToDO: Timeout for starting simulation

SimulationServer::SimulationServer()
    : Node("simulation_server"), action_in_progress_(false),
      gz_node_(std::make_shared<gz::transport::Node>()) {
  // Declare parameters
  this->declare_parameter<bool>("hide_simulation_process_output", true);
  this->get_parameter("hide_simulation_process_output",
                      hide_simulation_process_output_);

  this->declare_parameter<bool>("headless_mode", false);
  this->get_parameter("headless_mode", headless_mode_);

  this->declare_parameter<std::string>("px4_path", "");
  this->get_parameter("px4_path", px4_path_);

  this->declare_parameter<std::string>("xrce_agent_path", "");
  this->get_parameter("xrce_agent_path", xrce_agent_path_);

  RCLCPP_INFO(this->get_logger(), "Simulation process output hiding: %s",
              hide_simulation_process_output_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "Headless mode: %s",
              headless_mode_ ? "enabled" : "disabled");
  
  // Validate PX4 executable path
  if (!px4_path_.empty()) {
    if (std::filesystem::exists(px4_path_) && 
        (std::filesystem::status(px4_path_).permissions() & 
         std::filesystem::perms::owner_exec) != std::filesystem::perms::none) {
      validated_px4_path_ = px4_path_;
      RCLCPP_INFO(this->get_logger(), "Custom PX4 executable found at: %s", 
                  validated_px4_path_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), 
                  "Custom PX4 executable not found or not executable at: %s", 
                  px4_path_.c_str());
    }
  }
  
  // Validate MicroXRCE-DDS Agent executable path
  if (!xrce_agent_path_.empty()) {
    if (std::filesystem::exists(xrce_agent_path_) && 
        (std::filesystem::status(xrce_agent_path_).permissions() & 
         std::filesystem::perms::owner_exec) != std::filesystem::perms::none) {
      validated_xrce_agent_path_ = xrce_agent_path_;
      RCLCPP_INFO(this->get_logger(), "Custom MicroXRCE-DDS Agent found at: %s", 
                  validated_xrce_agent_path_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), 
                  "Custom MicroXRCE-DDS Agent not found or not executable at: %s", 
                  xrce_agent_path_.c_str());
    }
  }

  // Initialize the clock publisher
  clock_publisher_ =
      this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  start_action_server_ = rclcpp_action::create_server<StartSim>(
      this, "start_simulation",
      std::bind(&SimulationServer::handle_start_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&SimulationServer::handle_start_cancel, this,
                std::placeholders::_1),
      std::bind(&SimulationServer::handle_start_accepted, this,
                std::placeholders::_1));

  stop_action_server_ = rclcpp_action::create_server<StopSim>(
      this, "stop_simulation",
      std::bind(&SimulationServer::handle_stop_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&SimulationServer::handle_stop_cancel, this,
                std::placeholders::_1),
      std::bind(&SimulationServer::handle_stop_accepted, this,
                std::placeholders::_1));
}

// Callback for Gazebo clock messages
void SimulationServer::on_clock_message(const gz::msgs::Clock &msg) {
  // Convert Gazebo time to ROS time
  rosgraph_msgs::msg::Clock ros_clock_msg;
  ros_clock_msg.clock.sec = msg.sim().sec();
  ros_clock_msg.clock.nanosec = msg.sim().nsec();

  // Publish to ROS clock topic
  clock_publisher_->publish(ros_clock_msg);
}

// StartSimulation handlers
rclcpp_action::GoalResponse SimulationServer::handle_start_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const StartSim::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received StartSimulation goal");

  // Check if another action is already in progress
  if (action_in_progress_.load()) {
    RCLCPP_WARN(this->get_logger(),
                "Another action is already in progress. Try again later.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (is_simulation_running()) {
    RCLCPP_WARN(this->get_logger(), "Simulation already running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check for duplicate model names early
  std::set<std::string> model_names;
  for (const auto &model_info : goal->models) {
    if (model_names.count(model_info.model_name) > 0) {
      // Duplicate model name found
      RCLCPP_WARN(this->get_logger(),
                  "Rejecting goal: Duplicate model name found: %s",
                  model_info.model_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    model_names.insert(model_info.model_name);
  }

  // Set flag to indicate an action is now in progress
  action_in_progress_.store(true);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimulationServer::handle_start_cancel(
    const std::shared_ptr<GoalHandleStartSim>) {
  return rclcpp_action::CancelResponse::REJECT;
}

void SimulationServer::handle_start_accepted(
    const std::shared_ptr<GoalHandleStartSim> goal_handle) {
  std::thread{std::bind(&SimulationServer::execute_start, this, goal_handle)}
      .detach();
}

void SimulationServer::execute_start(
    const std::shared_ptr<GoalHandleStartSim> goal_handle) {
  auto result = std::make_shared<StartSim::Result>();
  auto goal = goal_handle->get_goal();

  RCLCPP_INFO(this->get_logger(), "Launching Gazebo simulation...");

  // Reset any existing PIDs
  gazebo_pid_ = -1;
  px4_pids_.clear();

  // 1. Launch Gazebo
  std::vector<std::string> gz_args = {"sim"};

  // Add headless flag if enabled
  if (headless_mode_) {
    gz_args.push_back("-s");
  }

  // Add world file
  gz_args.push_back("-r");
  gz_args.push_back(goal->world);

  gazebo_pid_ = launch_process("gz", gz_args, "Gazebo");

  if (gazebo_pid_ < 0) {
    result->message = "Failed to launch Gazebo process";
    goal_handle->abort(result);
    action_in_progress_.store(false);
    return;
  } else {

    // 2. Wait for Gazebo to be fully initialized
    RCLCPP_INFO(this->get_logger(), "Waiting for Gazebo to be ready...");
    if (!wait_for_gazebo_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Gazebo failed to initialize properly");
      // Kill Gazebo since it's not working properly
      terminate_process(gazebo_pid_, "Gazebo");
      gazebo_pid_ = -1;

      result->message = "Gazebo failed to initialize properly";
      goal_handle->abort(result);
      action_in_progress_.store(false);
      return;
    }

    // Subscribe to Gazebo clock topic and publish to ROS clock
    RCLCPP_INFO(this->get_logger(),
                "Setting up clock topic forwarding from Gazebo to ROS...");
    if (!gz_node_->Subscribe("/clock", &SimulationServer::on_clock_message,
                             this)) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to subscribe to Gazebo clock topic");
      // Stop simulation if we can't forward the clock
      shutdown_simulation();
      result->message = "Failed to subscribe to Gazebo clock topic";
      goal_handle->abort(result);
      action_in_progress_.store(false);
      return;
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully subscribed to Gazebo clock topic");
    }

    // 3. Spawn all models using EntityFactory service
    for (const auto &model_info : goal->models) {
      RCLCPP_INFO(this->get_logger(), "Spawning model: %s",
                  model_info.model_name.c_str());

      if (!spawn_model(model_info.model_name, model_info.model_dir,
                       model_info.x, model_info.y, model_info.z)) {
        result->message = "Failed to spawn model: " + model_info.model_name;
        shutdown_simulation();
        goal_handle->abort(result);
        action_in_progress_.store(false);
        return;
      }
    }

    // 4. Get path to PX4 executable
    std::string pkg_px4_sim =
        ament_index_cpp::get_package_share_directory("robotsix_px4_simulation");
    std::string px4_path;
    
    // Use validated custom PX4 path if available, otherwise check the default
    if (!validated_px4_path_.empty()) {
      px4_path = validated_px4_path_;
      RCLCPP_INFO(this->get_logger(), "Using custom PX4 executable: %s", 
                  px4_path.c_str());
    } else {
      // Check if default PX4 executable exists and is executable
      std::string default_px4_path = pkg_px4_sim + "/px4_sitl_default/bin/px4";
      
      if (std::filesystem::exists(default_px4_path) && 
          (std::filesystem::status(default_px4_path).permissions() & 
           std::filesystem::perms::owner_exec) != std::filesystem::perms::none) {
        px4_path = default_px4_path;
        RCLCPP_INFO(this->get_logger(), "Using default PX4 executable: %s", 
                    px4_path.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), 
                    "Default PX4 executable not found or not executable at: %s", 
                    default_px4_path.c_str());
        result->message = "PX4 executable not found or not executable";
        goal_handle->abort(result);
        action_in_progress_.store(false);
        return;
      }
    }

    // 5. Launch PX4 for each model
    int instance_id = 0; // Start with instance ID 0
    for (const auto &model_info : goal->models) {
      // Set environment variables for PX4
      setenv("PX4_SYS_AUTOSTART",
             std::to_string(model_info.px4_parameters).c_str(), 1);
      setenv("PX4_GZ_MODEL_NAME", model_info.model_name.c_str(), 1);
      setenv("PX4_UXRCE_DDS_NS", model_info.model_name.c_str(), 1);

      // Launch PX4
      std::vector<std::string> px4_args = {
          pkg_px4_sim + "/px4_sitl_default/etc",
          "-s",
          pkg_px4_sim + "/px4_sitl_default/etc/init.d-posix/rcS",
          "-w",
          pkg_px4_sim + "/px4_sitl_default",
          "-i",
          std::to_string(instance_id)}; // Use integer instance ID
      pid_t px4_pid = launch_process(px4_path, px4_args,
                                     "PX4 for " + model_info.model_name);
      if (px4_pid < 0) {
        result->message =
            "Failed to launch PX4 process for model: " + model_info.model_name;
        shutdown_simulation();
        goal_handle->abort(result);
        action_in_progress_.store(false);
        return;
      }
      px4_pids_.push_back(px4_pid);
      instance_id++; // Increment instance ID for the next drone
    }

    // 6. Launch MicroXRCEAgent
    std::string xrce_agent_path;
    
    // Use validated custom MicroXRCE-DDS Agent path if available, otherwise check the default
    if (!validated_xrce_agent_path_.empty()) {
      xrce_agent_path = validated_xrce_agent_path_;
      RCLCPP_INFO(this->get_logger(), "Using custom MicroXRCE-DDS Agent: %s", 
                  xrce_agent_path.c_str());
    } else {
      // Check if default MicroXRCE-DDS Agent executable exists and is executable
      std::string default_xrce_agent_path = pkg_px4_sim + "/bin/MicroXRCEAgent";
      
      if (std::filesystem::exists(default_xrce_agent_path) && 
          (std::filesystem::status(default_xrce_agent_path).permissions() & 
           std::filesystem::perms::owner_exec) != std::filesystem::perms::none) {
        xrce_agent_path = default_xrce_agent_path;
        RCLCPP_INFO(this->get_logger(), "Using default MicroXRCE-DDS Agent: %s", 
                    xrce_agent_path.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), 
                    "Default MicroXRCE-DDS Agent not found or not executable at: %s", 
                    default_xrce_agent_path.c_str());
        result->message = "MicroXRCE-DDS Agent executable not found or not executable";
        goal_handle->abort(result);
        action_in_progress_.store(false);
        return;
      }
    }
    std::vector<std::string> agent_args = {"udp4", "-p", "8888"};
    xrce_agent_pid_ =
        launch_process(xrce_agent_path, agent_args, "MicroXRCEAgent");
    if (xrce_agent_pid_ < 0) {
      result->message = "Failed to launch MicroXRCEAgent process";
      shutdown_simulation();
      goal_handle->abort(result);
      action_in_progress_.store(false);
      return;
    }

    // Collect all model names for topic checking
    std::vector<std::string> model_names;
    for (const auto &model_info : goal->models) {
      model_names.push_back(model_info.model_name);
    }

    // Wait for drone topics to be available (PX4 to ROS2 connection)
    RCLCPP_INFO(this->get_logger(),
                "Waiting for PX4 to ROS2 connection for all drones...");
    if (!wait_for_drone_topics(model_names)) {
      result->message = "Failed to detect ROS2 topics for all drones";
      shutdown_simulation();
      goal_handle->abort(result);
      action_in_progress_.store(false);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Simulation started successfully");
    goal_handle->succeed(result);
    action_in_progress_.store(false);
  }
}

// StopSimulation handlers
rclcpp_action::GoalResponse
SimulationServer::handle_stop_goal(const rclcpp_action::GoalUUID &,
                                   std::shared_ptr<const StopSim::Goal>) {
  RCLCPP_INFO(this->get_logger(), "Received StopSimulation goal");

  // Check if another action is already in progress
  if (action_in_progress_.load()) {
    RCLCPP_WARN(this->get_logger(),
                "Another action is already in progress. Try again later.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!is_simulation_running()) {
    RCLCPP_WARN(this->get_logger(), "No simulation is currently running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Set flag to indicate an action is now in progress
  action_in_progress_.store(true);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SimulationServer::handle_stop_cancel(const std::shared_ptr<GoalHandleStopSim>) {
  return rclcpp_action::CancelResponse::REJECT;
}

void SimulationServer::handle_stop_accepted(
    const std::shared_ptr<GoalHandleStopSim> goal_handle) {
  std::thread{std::bind(&SimulationServer::execute_stop, this, goal_handle)}
      .detach();
}

bool SimulationServer::is_simulation_running() {
  // Check if Gazebo, MicroXRCEAgent, or any PX4 process is running
  if (gazebo_pid_ <= 0 && xrce_agent_pid_ <= 0 && px4_pids_.empty()) {
    return false;
  }

  bool gazebo_running = false;
  bool any_px4_running = false;
  bool xrce_agent_running = false;

  // Check Gazebo process if we have a valid PID
  if (gazebo_pid_ > 0) {
    int status;
    pid_t result = waitpid(gazebo_pid_, &status, WNOHANG);
    gazebo_running = (result == 0); // Still running if waitpid returns 0
  }

  // Check if any PX4 process is running
  for (const auto &pid : px4_pids_) {
    if (pid > 0) {
      int status;
      pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == 0) { // Still running if waitpid returns 0
        any_px4_running = true;
        break;
      }
    }
  }

  // Check MicroXRCEAgent process if we have a valid PID
  if (xrce_agent_pid_ > 0) {
    int status;
    pid_t result = waitpid(xrce_agent_pid_, &status, WNOHANG);
    xrce_agent_running = (result == 0); // Still running if waitpid returns 0
  }

  // Simulation is running if at least one process is still alive
  return gazebo_running || any_px4_running || xrce_agent_running;
}

// Clean shutdown of simulation processes
bool SimulationServer::shutdown_simulation() {
  RCLCPP_INFO(this->get_logger(),
              "Attempting to cleanly shut down simulation...");

  bool success = true;

  // Stop publishing the clock if we were doing so
  if (gz_node_) {
    gz_node_->Unsubscribe("/clock");
  }
  RCLCPP_INFO(this->get_logger(), "Stopped forwarding Gazebo clock to ROS");

  // First terminate all PX4 instances, then MicroXRCEAgent, then Gazebo
  for (auto &pid : px4_pids_) {
    if (pid > 0) {
      success &= terminate_process(pid, "PX4");
      pid = -1;
    }
  }
  px4_pids_.clear();

  if (xrce_agent_pid_ > 0) {
    success &= terminate_process(xrce_agent_pid_, "MicroXRCEAgent");
    xrce_agent_pid_ = -1;
  }

  if (gazebo_pid_ > 0) {
    success &= terminate_process(gazebo_pid_, "Gazebo");
    gazebo_pid_ = -1;
  }

  return success;
}

// Terminate a process with increasing signal severity
bool SimulationServer::terminate_process(pid_t pid, const char *process_name) {
  if (pid <= 0) {
    return true; // No process to terminate
  }

  // Check if process is still running
  int status;
  pid_t result = waitpid(pid, &status, WNOHANG);
  if (result != 0) {
    RCLCPP_INFO(this->get_logger(), "%s process (PID %d) already terminated",
                process_name, pid);
    return true;
  }

  // Try signals in order: SIGINT -> SIGTERM -> SIGKILL
  for (int signal : {SIGINT, SIGTERM, SIGKILL}) {
    const char *signal_str =
        (signal == SIGINT) ? "SIGINT"
                           : ((signal == SIGTERM) ? "SIGTERM" : "SIGKILL");

    RCLCPP_INFO(this->get_logger(), "Sending %s to %s process (PID %d)",
                signal_str, process_name, pid);

    if (kill(pid, signal) != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to send %s to %s process (PID %d): %s", signal_str,
                  process_name, pid, strerror(errno));
      continue;
    }

    // Wait for process to terminate (up to 3 seconds)
    for (int i = 0; i < 6; i++) {
      result = waitpid(pid, &status, WNOHANG);
      if (result != 0) {
        RCLCPP_INFO(this->get_logger(), "%s process terminated after %s",
                    process_name, signal_str);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  // If we get here, even SIGKILL didn't work
  RCLCPP_ERROR(this->get_logger(),
               "Failed to terminate %s process (PID %d), even with SIGKILL",
               process_name, pid);
  return false;
}

void SimulationServer::execute_stop(
    const std::shared_ptr<GoalHandleStopSim> goal_handle) {
  auto result = std::make_shared<StopSim::Result>();

  RCLCPP_INFO(this->get_logger(), "Stopping Gazebo simulation...");

  bool success = shutdown_simulation();

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Gazebo simulation stopped successfully.");
    goal_handle->succeed(result);
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to completely stop Gazebo simulation.");
    result->message = "Failed to completely stop Gazebo simulation.";
    goal_handle->abort(result);
  }

  // Reset the action in progress flag
  action_in_progress_.store(false);
}

pid_t SimulationServer::launch_process(const std::string &executable_path,
                                       const std::vector<std::string> &args,
                                       const std::string &process_name) {
  pid_t child_pid = fork();

  if (child_pid < 0) {
    // Fork failed
    RCLCPP_ERROR(this->get_logger(), "Failed to fork %s process: %s",
                 process_name.c_str(), strerror(errno));
    return -1;
  } else if (child_pid == 0) {
    // Child process
    // Redirect stdout and stderr to /dev/null if hiding is enabled
    if (hide_simulation_process_output_) {
      int null_fd = open("/dev/null", O_WRONLY);
      if (null_fd >= 0) {
        dup2(null_fd, STDOUT_FILENO);
        dup2(null_fd, STDERR_FILENO);
        close(null_fd);
      }
    }

    // Prepare arguments for execvp
    std::vector<char *> exec_args;
    exec_args.push_back(const_cast<char *>(executable_path.c_str()));

    for (const auto &arg : args) {
      exec_args.push_back(const_cast<char *>(arg.c_str()));
    }

    // Add nullptr as last argument (required by execvp)
    exec_args.push_back(nullptr);

    // Execute the command
    execvp(exec_args[0], exec_args.data());

    // If execvp returns, it failed
    RCLCPP_ERROR(this->get_logger(), "Failed to execute %s: %s",
                 process_name.c_str(), strerror(errno));
    exit(EXIT_FAILURE);
  }

  // Parent process
  RCLCPP_INFO(this->get_logger(), "Started %s process with PID: %d",
              process_name.c_str(), child_pid);

  return child_pid;
}

bool SimulationServer::wait_for_drone_topics(
    const std::vector<std::string> &model_names, int timeout_seconds) {
  RCLCPP_INFO(this->get_logger(), "Checking for drone ROS2 topics...");

  auto start_time = std::chrono::steady_clock::now();

  // Keep track of which drones we've found topics for
  std::map<std::string, bool> drone_topics_found;
  for (const auto &name : model_names) {
    drone_topics_found[name] = false;
  }

  // How many drones we need to find topics for
  int drones_to_find = model_names.size();

  while (drones_to_find > 0) {
    // Check if we've exceeded the timeout
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       current_time - start_time)
                       .count();

    if (elapsed > timeout_seconds) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Timeout waiting for drone ROS2 topics. Missing topics for:");
      for (const auto &pair : drone_topics_found) {
        if (!pair.second) {
          RCLCPP_ERROR(this->get_logger(), "  - %s", pair.first.c_str());
        }
      }
      return false;
    }

    // Get all available topics
    auto topic_names_and_types = this->get_topic_names_and_types();

    // Check each drone we haven't found yet
    for (auto &pair : drone_topics_found) {
      if (pair.second)
        continue; // Skip drones we've already found

      std::string topic_name = "/" + pair.first + "/fmu/out/timesync_status";

      // Check if the topic exists
      for (const auto &[name, types] : topic_names_and_types) {
        if (name == topic_name) {
          RCLCPP_INFO(this->get_logger(), "Found ROS2 topic for drone %s",
                      pair.first.c_str());
          pair.second = true;
          drones_to_find--;
          break;
        }
      }
    }

    if (drones_to_find > 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for %d drone ROS2 topics to become available...",
                  drones_to_find);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  RCLCPP_INFO(this->get_logger(), "All drone ROS2 topics are available");
  return true;
}

bool SimulationServer::wait_for_gazebo_ready(int timeout_seconds) {
  RCLCPP_INFO(this->get_logger(), "Checking for Gazebo world topics...");

  gz::transport::Node node;

  auto start_time = std::chrono::steady_clock::now();
  bool world_ready = false;

  while (!world_ready) {
    // Check if we've exceeded the timeout
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       current_time - start_time)
                       .count();

    if (elapsed > timeout_seconds) {
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout waiting for Gazebo world to be ready");
      return false;
    }

    // Get all topics and look for scene/info topic
    std::vector<std::string> topics;
    node.TopicList(topics);

    for (const auto &topic : topics) {
      // Look for pattern /world/{world_name}/scene/info
      if (topic.find("/world/") == 0 &&
          topic.find("/scene/info") != std::string::npos) {
        world_ready = true;
      }
    }

    if (!world_ready) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for Gazebo world to be ready...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return true;
}

std::string SimulationServer::find_model_path(const std::string &model_dir) {
  // TODO: If not found in absolute path, remove the potential / in front
  // First check if the model_dir is an absolute path that exists
  std::filesystem::path dir_path(model_dir);
  if (dir_path.is_absolute() && std::filesystem::is_directory(dir_path)) {
    return model_dir;
  }

  // Get the GZ_SIM_RESOURCE_PATH environment variable
  const char *gz_resource_path = std::getenv("GZ_SIM_RESOURCE_PATH");
  if (!gz_resource_path) {
    RCLCPP_WARN(this->get_logger(),
                "GZ_SIM_RESOURCE_PATH environment variable not set");
    return "";
  }

  // Split the path by colons
  std::string path_str(gz_resource_path);
  std::istringstream path_stream(path_str);
  std::string path_component;

  // Check each resource path component
  while (std::getline(path_stream, path_component, ':')) {
    if (path_component.empty()) {
      continue;
    }

    // Create the potential full path
    std::filesystem::path full_path =
        std::filesystem::path(path_component) / model_dir;

    // Check if this path exists and is a directory
    if (std::filesystem::is_directory(full_path)) {
      RCLCPP_INFO(this->get_logger(), "Found model directory at: %s",
                  full_path.string().c_str());
      return full_path.string();
    }
  }

  RCLCPP_ERROR(this->get_logger(),
               "Could not find model directory '%s' in any "
               "GZ_SIM_RESOURCE_PATH location",
               model_dir.c_str());
  return "";
}

std::string SimulationServer::process_model_dir(const std::string &model_dir) {
  std::string absolute_model_path = find_model_path(model_dir);
  if (absolute_model_path.empty()) {
    return "";
  }

  std::filesystem::path model_path(absolute_model_path);

  // Create a temporary directory as a copy of the original
  char tmp_dir_template[] = "/tmp/model_XXXXXX";
  char *tmp_dir_name = mkdtemp(tmp_dir_template);
  if (tmp_dir_name == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create temporary directory: %s",
                 strerror(errno));
    return "";
  }

  std::string tmp_model_dir(tmp_dir_name);
  RCLCPP_INFO(this->get_logger(), "Created temporary directory: %s",
              tmp_model_dir.c_str());

  // Copy all files from the original model directory to the temporary directory
  try {
    for (const auto &entry : std::filesystem::directory_iterator(model_path)) {
      std::filesystem::path dest =
          std::filesystem::path(tmp_model_dir) / entry.path().filename();
      std::filesystem::copy(entry.path(), dest,
                            std::filesystem::copy_options::overwrite_existing);
    }
  } catch (const std::filesystem::filesystem_error &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to copy model files: %s",
                 e.what());
    return "";
  }

  // Look for .sdf.xacro files in the temporary directory
  std::string xacro_file;
  for (const auto &entry : std::filesystem::directory_iterator(tmp_model_dir)) {
    std::string file_path = entry.path().string();
    if (file_path.length() >= 10 &&
        file_path.substr(file_path.length() - 10) == ".sdf.xacro") {
      xacro_file = file_path;
      RCLCPP_INFO(this->get_logger(),
                  "Found xacro file in temporary directory: %s",
                  xacro_file.c_str());
      break;
    }
  }

  // If we found a .sdf.xacro file, process it
  if (!xacro_file.empty()) {
    std::filesystem::path xacro_path(xacro_file);
    std::string model_name =
        xacro_path.stem().stem().string(); // Remove both .sdf.xacro extensions

    // Create the output file in the temporary directory with the same name but
    // without .xacro
    std::string output_file = tmp_model_dir + "/" + model_name + ".sdf";

    // Build command to run xacro
    std::vector<std::string> xacro_args = {xacro_file, "-o", output_file};

    // Run xacro command
    pid_t xacro_pid = launch_process("xacro", xacro_args, "xacro");
    if (xacro_pid <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to launch xacro process");
      return "";
    }

    // Wait for xacro to complete
    int status;
    if (waitpid(xacro_pid, &status, 0) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to wait for xacro process: %s",
                   strerror(errno));
      return "";
    }

    // Check if xacro command was successful
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Xacro conversion failed with status code: %d",
                   WIFEXITED(status) ? WEXITSTATUS(status) : -1);
      return "";
    }

    // Check if the output file exists
    if (!std::filesystem::exists(output_file)) {
      RCLCPP_ERROR(this->get_logger(), "Xacro output file not created: %s",
                   output_file.c_str());
      return "";
    }

    // Remove the xacro file from the temporary directory
    try {
      std::filesystem::remove(xacro_file);
    } catch (const std::filesystem::filesystem_error &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to remove xacro file: %s",
                  e.what());
      // Continue anyway, this is not critical
    }

    RCLCPP_INFO(this->get_logger(), "Successfully converted xacro to SDF: %s",
                output_file.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "No .sdf.xacro file found in directory: %s",
                model_dir.c_str());
  }

  return tmp_model_dir;
}

bool SimulationServer::spawn_model(const std::string &model_name,
                                   const std::string &model_dir, float x,
                                   float y, float z, int timeout_seconds) {
  RCLCPP_INFO(this->get_logger(), "Spawning model %s at position [%f, %f, %f]",
              model_name.c_str(), x, y, z);

  // Find the absolute path to the model directory
  std::string absolute_model_path = find_model_path(model_dir);
  if (absolute_model_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find model directory: %s",
                 model_dir.c_str());
    return false;
  }

  // Process model directory (convert xacro if needed) and get the temporary
  // directory path
  std::string tmp_model_dir = process_model_dir(absolute_model_path);
  if (tmp_model_dir.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process model directory: %s",
                 absolute_model_path.c_str());
    return false;
  }

  gz::transport::Node node;
  gz::msgs::EntityFactory req;

  // Set model properties with the temporary path
  req.set_sdf_filename(tmp_model_dir);
  req.set_name(model_name);

  // Set model pose
  auto pose = req.mutable_pose();
  auto position = pose->mutable_position();
  position->set_x(x);
  position->set_y(y);
  position->set_z(z);

  // Convert request message to string
  std::string req_str;
  if (!req.SerializeToString(&req_str)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to serialize request for model %s",
                 model_name.c_str());
    return false;
  }

  // Call the service with a timeout (blocking call)
  std::string service = "/world/default/create";
  std::string rep_str;
  bool call_result = false;

  const unsigned int timeout_ms = 1000; // 1 second timeout
  bool transport_result =
      node.RequestRaw(service, req_str, "gz.msgs.EntityFactory",
                      "gz.msgs.Boolean", timeout_ms, rep_str, call_result);

  if (!transport_result || !call_result) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call EntityFactory service for model %s",
                 model_name.c_str());
    return false;
  }

  // Parse response
  gz::msgs::Boolean rep;
  if (!rep.ParseFromString(rep_str)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse response for model %s",
                 model_name.c_str());
    return false;
  }

  if (!rep.data()) {
    RCLCPP_ERROR(this->get_logger(),
                 "EntityFactory service failed to spawn model %s",
                 model_name.c_str());
    return false;
  }

  // Wait for the model to actually appear in the Gazebo world
  RCLCPP_INFO(this->get_logger(), "Waiting for model %s to spawn in Gazebo...",
              model_name.c_str());

  bool model_spawned = false;
  auto start_time = std::chrono::steady_clock::now();

  while (!model_spawned) {
    // Check if we've exceeded the timeout
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       current_time - start_time)
                       .count();

    if (elapsed > timeout_seconds) {
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout waiting for model %s to spawn in Gazebo",
                   model_name.c_str());
      return false;
    }

    // Get all topics and look for topics related to this model
    std::vector<std::string> topics;
    node.TopicList(topics);

    const std::string model_topic_pattern = "/model/" + model_name + "/";

    for (const auto &topic : topics) {
      if (topic.find(model_topic_pattern) != std::string::npos) {
        model_spawned = true;
        break;
      }
    }

    if (!model_spawned) {
      RCLCPP_DEBUG(this->get_logger(), "Waiting for model %s to appear...",
                   model_name.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Successfully spawned model %s and verified its presence",
              model_name.c_str());

  // Clean up the temporary directory only after model is confirmed spawned
  try {
    std::filesystem::remove_all(tmp_model_dir);
    RCLCPP_DEBUG(this->get_logger(), "Removed temporary directory: %s",
                 tmp_model_dir.c_str());
  } catch (const std::filesystem::filesystem_error &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to remove temporary directory: %s",
                e.what());
    // Continue anyway, this is not critical
  }
  return true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimulationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
