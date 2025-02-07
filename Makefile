# Detect architecture and set appropriate image
ARCH := $(shell uname -m)
ifeq ($(ARCH), arm64)
  CONTAINER_NAME := arm64v8/ros:humble
else
  CONTAINER_NAME := osrf/ros:humble-desktop
endif

build:
	docker build . -t ${IMAGE_NAME}

novnc:
	docker run -d --rm --net=ros \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc -p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --name ${NAME} \
	--net=ros \
	--env="DISPLAY=novnc:0.0" \
	-v ./:/CPE416/:Z \
	-e USER=${USER} \
	${CONTAINER_NAME}

arch:
	@echo "Make will pull the following image: ${CONTAINER_NAME}"
