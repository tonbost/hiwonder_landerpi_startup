# LanderPi ROS2 Docker Image with Hailo-8 Support
# Based on ROS2 Humble with Aurora 930 camera and Hailo-8 NPU drivers

FROM ros:humble-ros-base

# Install base dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    python3-smbus \
    i2c-tools \
    libusb-1.0-0-dev \
    libudev-dev \
    curl \
    gnupg \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-angles \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-vision-msgs \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Install YOLOv11 dependencies (numpy<2 required for cv_bridge compatibility)
# pyzmq is used for the Hailo bridge to communicate with host inference server
RUN pip3 install --no-cache-dir "numpy<2" ultralytics onnxruntime pyzmq

WORKDIR /ros2_ws
CMD ["bash"]
