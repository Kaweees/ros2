ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM $BASE_IMAGE

ARG BASE_IMAGE

# Install additional dependencies
RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /CPE416

# Add any additional ROS2 packages you need
RUN apt-get update && apt-get install -y \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  && rm -rf /var/lib/apt/lists/*

# Source ROS2 in every new shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Keep container running
CMD ["bash"]
