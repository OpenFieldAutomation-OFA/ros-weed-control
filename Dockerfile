# credit: https://github.com/sea-bass/turtlebot3_behavior_demos

# ROS distribution
ARG ROS_DISTRO=humble

##############
# Base Image #
##############
FROM ros:$ROS_DISTRO-ros-base AS base

# Create underlay workspace
WORKDIR /opt/underlay/src

# Install external dependencies
COPY dependencies.repos .
RUN vcs import  < dependencies.repos

# Build underlay workspace
WORKDIR /opt/underlay
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    && rosdep install -y --from-paths src --ignore-src \
    && colcon build \
    && rm -rf /var/lib/apt/lists/*

# Define user
ARG UID=1000
ARG GID=1000
ARG USERNAME=ros

# Create user and overlay workspace
RUN groupadd --gid $GID $USERNAME \
    && useradd -s /bin/bash --uid $UID --gid $GID \
    -m -d /opt/overlay $USERNAME
WORKDIR /opt/overlay

# Copy source code
COPY --chown=$UID:$GID . src

# Install rosdep dependencies
RUN apt-get update \
    && rosdep install -y --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

####################
# Production Image #
####################
FROM base AS prod

# Change entrypoint
RUN sed --in-place \
    's|^source .*|source "/opt/overlay/install/setup.bash"|' \
    /ros_entrypoint.sh

USER $USERNAME

# Build overlay
RUN . /opt/underlay/install/setup.sh \
    && colcon build

USER root

#####################
# Development Image #
#####################
FROM base AS dev

# Install apt packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    sudo \
    neovim \
    ros-$ROS_DISTRO-turtlesim
    # && rm -rf /var/lib/apt/lists/*

# Add sudo privileges to user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# Add ROS setup file to bashrc
RUN echo "source /opt/underlay/install/setup.bash" >> ~/.bashrc

