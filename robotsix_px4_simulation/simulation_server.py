#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""
Simulation Server Node - Pure Python Implementation
Delegates all process management to launch files
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from robotsix_px4_sim_interface.action import StartSimulation, StopSimulation

import threading
import json
import asyncio
import multiprocessing
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, Shutdown
import launch
from ament_index_python.packages import get_package_share_directory
import os


class Ros2LaunchParent:
    """Launch parent class to manage ROS2 launch service in a separate process"""
    
    def start(self, launch_description: LaunchDescription):
        """Start the launch service in a separate process"""
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process,
            args=(self._stop_event, launch_description),
            daemon=True
        )
        self._process.start()

    def shutdown(self):
        """Shutdown the launch service and wait for process to complete"""
        if hasattr(self, '_stop_event') and hasattr(self, '_process'):
            self._stop_event.set()
            self._process.join(timeout=10)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=5)

    def _run_process(self, stop_event, launch_description):
        """Run the launch service in a separate process"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
        if not launch_task.done():
            asyncio.ensure_future(launch_service.shutdown(), loop=loop)
            loop.run_until_complete(launch_task)
        loop.close()
    
    def is_running(self):
        """Check if the process is still running"""
        if hasattr(self, '_process'):
            return self._process.is_alive()
        return False


class SimulationServer(Node):
    def __init__(self):
        super().__init__("simulation_server")
        
        # Callback group for concurrent handling
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('hide_simulation_process_output', True)
        self.declare_parameter('headless_mode', False)
        self.declare_parameter('px4_dir', '')
        self.declare_parameter('xrce_agent_path', '')
        
        # Get parameters
        self.hide_output = self.get_parameter(
            'hide_simulation_process_output').value
        self.headless = self.get_parameter('headless_mode').value
        self.px4_dir = self.get_parameter('px4_dir').value
        self.xrce_agent_path = self.get_parameter('xrce_agent_path').value
        
        # Launch service management
        self.launch_parent = None
        
        # Path to simulation processes launch file
        pkg_share = get_package_share_directory('robotsix_px4_simulation')
        self.launch_file = os.path.join(
            pkg_share, 'launch', 'simulation_processes.launch.py')
        
        # Action servers
        self.start_action_server = ActionServer(
            self,
            StartSimulation,
            'start_simulation',
            execute_callback=self.execute_start,
            goal_callback=self.handle_start_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group
        )
        
        self.stop_action_server = ActionServer(
            self,
            StopSimulation,
            'stop_simulation',
            execute_callback=self.execute_stop,
            goal_callback=self.handle_stop_goal,
            cancel_callback=self.handle_cancel,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Simulation Server initialized")
        self.get_logger().info(f"Headless mode: {self.headless}")
        self.get_logger().info(f"PX4 directory: {self.px4_dir}")
        
    def is_simulation_running(self):
        """Check if simulation launch service is running"""
        return self.launch_parent is not None and self.launch_parent.is_running()
    
    def handle_start_goal(self, goal_request):
        """Handle start simulation goal"""
        self.get_logger().info("Received StartSimulation goal")
        
        if self.is_simulation_running():
            if goal_request.force_restart:
                self.get_logger().info(
                    "Simulation running - force restart requested")
            else:
                self.get_logger().warn(
                    "Simulation already running. Use force_restart=true")
                return GoalResponse.REJECT
        
        # Check for duplicate model names
        model_names = [m.model_name for m in goal_request.models]
        if len(model_names) != len(set(model_names)):
            self.get_logger().warn("Duplicate model names detected")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def handle_stop_goal(self, goal_request):
        """Handle stop simulation goal"""
        self.get_logger().info("Received StopSimulation goal")
        
        if not self.is_simulation_running():
            self.get_logger().warn("No simulation currently running")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def handle_cancel(self, goal_handle):
        """Handle cancellation (not supported)"""
        return CancelResponse.REJECT
    
    def execute_start(self, goal_handle):
        """Execute start simulation action"""
        goal = goal_handle.request
        result = StartSimulation.Result()
        
        try:
            # Handle force restart: stop existing simulation first
            if goal.force_restart and self.is_simulation_running():
                self.get_logger().info(
                    "Stopping current simulation for restart...")
                if not self.stop_simulation():
                    result.message = "Failed to stop existing simulation"
                    goal_handle.abort()
                    return result
                
                import time
                time.sleep(2)  # Wait for cleanup
            
            # Start simulation via launch file
            self.get_logger().info("Starting simulation...")
            if not self.start_simulation(goal):
                result.message = "Failed to start simulation"
                goal_handle.abort()
                return result
            
            self.get_logger().info("Simulation started successfully")
            result.message = "Simulation started"
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f"Error starting simulation: {str(e)}")
            result.message = str(e)
            self.stop_simulation()
            goal_handle.abort()

        return result
    
    def execute_stop(self, goal_handle):
        """Execute stop simulation action"""
        result = StopSimulation.Result()
        
        try:
            self.get_logger().info("Stopping simulation...")
            if self.stop_simulation():
                result.message = "Simulation stopped"
                goal_handle.succeed()
            else:
                result.message = "Failed to stop simulation"
                goal_handle.abort()
        
        return result
    
    def start_simulation(self, goal):
        """Start simulation by launching simulation_processes.launch.py"""
        try:
            # Convert models to JSON string for passing to launch file
            models_data = []
            for model in goal.models:
                models_data.append({
                    'model_name': model.model_name,
                    'model_dir': model.model_dir,
                    'x': model.x,
                    'y': model.y,
                    'z': model.z,
                    'px4_parameters': model.px4_parameters
                })
            models_json = json.dumps(models_data)
            
            # Prepare launch arguments
            launch_args = {
                'world': goal.world,
                'headless_mode': 'true' if self.headless else 'false',
                'hide_simulation_process_output':
                    'true' if self.hide_output else 'false',
                'px4_dir': self.px4_dir,
                'xrce_agent_path': self.xrce_agent_path,
                'models': models_json
            }
            
            # Create launch description
            launch_description_source = PythonLaunchDescriptionSource(
                self.launch_file)
            
            include_launch = IncludeLaunchDescription(
                launch_description_source,
                launch_arguments=launch_args.items()
            )
            
            ld = launch.LaunchDescription([include_launch])
            
            # Start launch service using Ros2LaunchParent
            self.launch_parent = Ros2LaunchParent()
            self.launch_parent.start(ld)
            
            self.get_logger().info("Launch service started")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to start simulation: {str(e)}")
            if self.launch_parent:
                self.launch_parent.shutdown()
                self.launch_parent = None
            return False
    
    def stop_simulation(self):
        """Stop simulation by shutting down launch service"""
        try:
            if self.launch_parent:
                self.get_logger().info("Shutting down launch service...")
                self.launch_parent.shutdown()
                self.launch_parent = None
                self.get_logger().info("Launch service stopped")
                return True
            return True
        except Exception as e:
            self.get_logger().error(f"Error stopping simulation: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = SimulationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_simulation_running():
            node.stop_simulation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()