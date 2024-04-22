# See all ros image tags here: https://hub.docker.com/_/ros
FROM ros:humble-ros-base

# GID and UID should be the same as the local user
ARG UID=1000
ARG GID=1000
ARG USERNAME=rosdev

# Install apt packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    sudo \
    neovim \
    && rm -rf /var/lib/apt/lists/*

# Create user with sudo privileges (https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
RUN groupadd --gid $GID $USERNAME \
    && useradd -s /bin/bash --uid $UID --gid $GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to user
USER $USERNAME

# Create workspace so that the user owns this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Add ROS setup file to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc