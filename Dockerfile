# Use ROS2 Humble Hawksbill as base image
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM $BASE_IMAGE
ARG BASE_IMAGE

# Install necessary packages
RUN apt-get update && apt-get install -y \
  ros-humble-turtlesim \
  stow \
  zsh \
  curl \
  git \
  wget \
  neovim \
  openssh-client \
  fzf \
  libnotify-bin \
  && rm -rf /var/lib/apt/lists/*

# Install the Gitstatus extension
RUN git clone --depth=1 https://github.com/romkatv/gitstatus.git ~/gitstatus

# Clone my zsh config and stow it
RUN git clone https://github.com/Kaweees/zsh.git ~/.config/zsh
RUN cd ~/.config/zsh && stow -t /root .

# Initialize ZSH (configuration will be loaded automatically)
RUN /bin/zsh -c "source /root/.zshrc 2>/dev/null"

# Set ZSH as default shell
RUN chsh -s /bin/zsh

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc

# Create and set working directory
WORKDIR /ros2_ws

# Keep container running
CMD ["zsh"]
