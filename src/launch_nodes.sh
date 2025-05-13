#!/bin/bash

# Launch multiple ROS2 nodes in a single tmux session
# Usage:
#    launch_nodes.sh launch_cmd1 [launch_cmd2 ... launch_cmdN]

#
#  Private Impl
#

setup(){
  # Set session name
  SESSION_NAME="mxcarkit"

  # Source the ROS2 environment
  source /opt/ros/humble/setup.zsh

  # Build the project
  colcon build

  # Source the project
  source install/setup.zsh

  # Kill any existing session with the same name
  tmux kill-session -t $SESSION_NAME 2>/dev/null || true

  # Start a new tmux session in detached mode
  tmux new-session -d -s $SESSION_NAME
}

# Track number of panes
PANE_COUNT=0

launch_node(){
  echo "Launching node $1"
  # First pane already exists, no need to create a new one
  if [ $PANE_COUNT -gt 0 ]; then
    tmux split-window -t $SESSION_NAME
  fi
  # Split the window horizontally
  tmux select-layout -t $SESSION_NAME even-horizontal
  # Run the node
  tmux send-keys -t $SESSION_NAME "ros2 run $1 $2" C-m
  # Increment the pane count
  PANE_COUNT=$((PANE_COUNT+1))
}

# Main script logic
set -e # Exit on error
if [ $# -eq 0 ]; then
  echo "Usage: $0 launch_cmd1 [launch_cmd2 ... launch_cmdN]"
  exit 1
else
  setup
  for arg in "$@"; do
    launch_node $arg
  done
  # Attach to the session
  tmux attach-session -t $SESSION_NAME
fi
