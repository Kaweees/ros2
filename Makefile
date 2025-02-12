# Makefile for
# Begin Variables Section

## Container Section: change these variables based on your container
# -----------------------------------------------------------------------------
# The container name.
TARGET := ros2

# The container network name.
NETWORK_NAME := ros
# The base container image based on your architecture.
ARCH := $(shell uname -m)
ifeq ($(ARCH), arm64) # ARM64
	CONTAINER_NAME := arm64v8/ros:humble
else # x86_64
	CONTAINER_NAME := osrf/ros:humble-desktop
endif

## Command Section: change these variables based on your commands
# -----------------------------------------------------------------------------
# Targets
.PHONY: all $(TARGET) pull build network bash novnc clean arch

# Rule to build the container
$(TARGET): all

# Default target: build the program
all: pull network build run

# Rule to pull the container image
pull:
	docker pull ${CONTAINER_NAME}
	docker pull theasp/novnc:latest

# Rule to build the Docker image
build: pull
	docker build . -t ${TARGET} --build-arg BASE_IMAGE=${CONTAINER_NAME}

# Rule to create the ROS network
network:
	docker network create ${NETWORK_NAME} 2>/dev/null || true

# Rule to run the container
run:
	docker run -d --rm --net=${NETWORK_NAME} \
		--env="DISPLAY=novnc:0.0" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="LIBGL_ALWAYS_INDIRECT=0" \
		-v $(PWD)/ros_ws/:/root/ros2_ws/:delegated \
		-v zsh_data:/root/.config/zsh \
		-v zsh_history:/root/.local/share/zinit \
		-w /root/ros2_ws \
		--name $(TARGET) \
		${TARGET} \
		tail -f /dev/null

start:
	docker start $(TARGET)

# Rule to run the VNC server
novnc: network build
	docker run -d --rm --net=${NETWORK_NAME} \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--env="OPENGL_DRIVER=1" \
	--name=novnc -p=8080:8080 \
	theasp/novnc:latest

# Rule to run the zsh shell
zsh:
	docker exec -it $(TARGET) zsh

# Rule to clean the container
clean:
	docker rm -f $(TARGET) 2>/dev/null || true
	docker rm -f novnc 2>/dev/null || true

# Rule to check the architecture
arch:
	@echo "Architecture: ${ARCH}"
	@echo "Container: ${CONTAINER_NAME}"
