ARG ROS_DISTRO=humble

# Use base image
FROM ros:$ROS_DISTRO-ros-base

# Prevent console from interacting with the user
ARG DEBIAN_FRONTEND=noninteractive

# Update package lists and install dependencies
RUN apt-get update -yqqq && apt-get install --no-install-recommends -yqqq \
    apt-utils \
    vim \
    git && \
    rm -rf /var/lib/apt/lists/*

# Set folder for RUNTIME_DIR
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# Python Dependencies
RUN apt-get update -yqqq && apt-get install --no-install-recommends -yqqq \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install additional ROS packages
RUN apt-get update -yqqq && apt-get install --no-install-recommends -yqqq \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-image-pipeline && \
    rm -rf /var/lib/apt/lists/*

# Using shell to use bash commands like 'source'
SHELL ["/bin/bash", "-c"]

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/ros2_ws
ENV WORKSPACE=$WORKSPACE

WORKDIR $WORKSPACE
