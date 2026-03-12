FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# System tools
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    nano \
    iputils-ping \
    net-tools \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN apt update &&  apt install -y \
    terminator
 
# ROS2 middleware
RUN apt update && apt install -y \
    ros-humble-rmw-cyclonedds-cpp

# Python dependencies
RUN pip3 install \
    pyserial \
    pygame

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
