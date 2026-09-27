FROM osrf/ros:jazzy-desktop-full

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    nano \
    cmake \
    build-essential \
    ninja-build \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    ros-jazzy-ros-gz \
    ros-jazzy-actuator-msgs \
    && rm -rf /var/lib/apt/lists/*

# Build Micro-XRCE-DDS-Agent (for communication between PX4 and ROS 2)
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    ldconfig && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# Build PX4-Autopilot
WORKDIR /
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    cd /PX4-Autopilot && \
    bash ./Tools/setup/ubuntu.sh --no-sim-tools

WORKDIR /workspace

# Automatically source the ROS 2 environment on login
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]
