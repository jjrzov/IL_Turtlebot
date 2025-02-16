ARCH := $(shell uname -m)

IMAGE_NAME := jjrzov/il_turtlebot:$(ARCH)

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
	-v $(shell pwd)/ros_ws:/IL_Turtlebot/ros_ws:Z \
	${IMAGE_NAME}

arch:
	@echo "Make will pull the following image: ${IMAGE_NAME}"
