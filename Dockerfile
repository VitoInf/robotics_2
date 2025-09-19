# This image creates a ROS2 Humble image with Gazebo Classic
# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop

# Install necessary dependencies for Nav2
RUN apt-get update && apt-get install -y nano\
    ros-humble-gazebo-*\
    ros-humble-turtlebot3-gazebo \
    ros-humble-tf-transformations \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=test_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Creating a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir  /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

 
# Setting up the sudo for the new User
RUN apt-get update \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Fix potential permission issues
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

# Going inside the home directory
WORKDIR /home/$USERNAME/
