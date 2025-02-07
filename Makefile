NETWORK_NAME := ros
NAME := ros2

# Detect architecture and set appropriate image
ARCH := $(shell uname -m)
ifeq ($(ARCH), arm64) # ARM64
	CONTAINER_NAME := arm64v8/ros:humble
else # x86_64
	CONTAINER_NAME := osrf/ros:humble-desktop
endif

pull:
	docker pull ${CONTAINER_NAME}
	docker pull theasp/novnc:latest

build: pull
	docker build . -t ${NAME} --build-arg BASE_IMAGE=${CONTAINER_NAME}

network:
	docker network create ${NETWORK_NAME} 2>/dev/null || true

novnc: network
	docker run -d --rm --net=${NETWORK_NAME} \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc -p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --name ${NAME} \
	--net=${NETWORK_NAME} \
	--env="DISPLAY=novnc:0.0" \
	-v ./:/CPE416/:Z \
	-e USER=${USER} \
	${CONTAINER_NAME}

arch:
	@echo "Architecture: ${ARCH}"
	@echo "Container: ${CONTAINER_NAME}"
