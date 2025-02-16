FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip \
    ros-dev-tools \
    ros-humble-ros-workspace \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
