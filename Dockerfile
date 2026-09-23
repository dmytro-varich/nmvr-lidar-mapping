FROM osrf/ros:jazzy-desktop-full

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Add new ros2 dependencies here
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Automatically source the ROS 2 environment on login
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]
