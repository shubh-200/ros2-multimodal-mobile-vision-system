# Official ROS 2 Jazzy Desktop image
FROM osrf/ros:jazzy-desktop

# Prevent interactive prompts during apt installations
ENV DEBIAN_FRONTEND=noninteractive

# Update system and install essential perception and build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    libopencv-dev \
    libpcl-dev \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create the workspace directory inside the container
WORKDIR /ros2_ws

# Copy only the source code (packages) into the container
COPY src /ros2_ws/src

# Install any remaining ROS 2 dependencies automatically
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Copy the custom entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]