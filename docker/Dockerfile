ARG BASE_IMAGE=robotsix/px4-sim-base:latest
FROM ${BASE_IMAGE}

# Get the packages from the repository
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/Robotsix-UAV/robotsix_px4_simulation.git && \
	git clone https://github.com/Robotsix-UAV/robotsix_px4_sim_interface.git

# Only for humble : clone https://github.com/gazebosim/ros_gz.git and checkout humble branch (necessary for Gazebo compatibility)
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
	git clone https://github.com/gazebosim/ros_gz.git && \
	cd ros_gz && \
	git checkout humble; \
fi

WORKDIR /root/ros2_ws

# Install dependencies
RUN apt-get update && \
	rosdep update && \
	rosdep install -y --from-paths src --ignore-src || true && \
	apt install -y libgflags-dev ros-${ROS_DISTRO}-actuator-msgs  && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build --event-handlers=console_direct+ && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*


# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sim"]
