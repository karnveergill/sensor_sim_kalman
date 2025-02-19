# Use an official ROS 2 base image (change "humble" to your desired ROS 2 distro)
FROM osrf/ros:humble-desktop

# Install necessary packages for GUI forwarding & ros2 colcon building
RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up display forwarding for macOS (XQuartz)
ENV DISPLAY=host.docker.internal:0
ENV QT_X11_NO_MITSHM=1

# Source ROS 2 setup script automatically & set ROS_DOMAIN_ID
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=94" >> ~/.bashrc

# Default command: Open a bash shell
CMD ["bash"]
