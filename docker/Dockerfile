FROM ros:jazzy-ros-base

# Install dependencies
RUN     apt-get update && \
	apt-get install -y curl python3-venv &&\
	curl https://packages.osrfoundation.org/gazebo.gpg | sudo gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
	apt-get update && \
	apt-get install -y gz-harmonic g++-11 libopencv-dev \
	&& apt remove -y curl \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

# Get the packages from the repository
ARG GIT_REF=main
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/Robotsix-UAV/robotsix_px4_simulation.git && \
	cd robotsix_px4_simulation && \
	git checkout ${GIT_REF} && \
	cd .. && \
	git clone https://github.com/Robotsix-UAV/robotsix_px4_sim_interface.git
WORKDIR /root/ros2_ws
# Install dependencies
RUN apt-get update && \
	rosdep update && \
	rosdep install -y --from-paths src --ignore-src && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*
# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
	export CXX=g++-11 && \
	colcon build --event-handlers=console_direct+ && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

# Create directories for custom Gazebo resources and PX4 parameters
RUN mkdir -p /custom_gz_resources /custom_px4_params

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sim"]
