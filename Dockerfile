# Use ROS2 Humble Hawksbill as base image
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM $BASE_IMAGE
ARG BASE_IMAGE

# Install necessary packages
RUN apt-get update && apt-get install -y \
  ros-humble-turtlesim \
  && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /ros2_ws

# Keep container running
CMD ["bash"]
