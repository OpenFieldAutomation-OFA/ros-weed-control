# credit: https://github.com/sea-bass/turtlebot3_behavior_demos

# ROS distribution
ARG ROS_DISTRO=humble

##############
# Base Image #
##############
FROM ros:$ROS_DISTRO-ros-base AS base

# Define user
ARG UID=1000
ARG GID=1000
ARG USERNAME=ros

# Create user
RUN groupadd --gid $GID $USERNAME \
    && useradd -s /bin/bash --uid $UID --gid $GID -m $USERNAME

# Create underlay workspace
WORKDIR /home/ros/underlay/src

# Clone external dependencies
COPY dependencies.repos .
RUN vcs import  < dependencies.repos

# Build underlay workspace
WORKDIR /home/ros/underlay
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    && rosdep install -y --from-paths src --ignore-src \
    && colcon build \
    && rm -rf /var/lib/apt/lists/*

# Create overlay workspace
RUN su $USERNAME -c "mkdir /home/ros/overlay"
WORKDIR /home/ros/overlay

####################
# Production Image #
####################
FROM base AS prod

# Copy source code
COPY --chown=$UID:$GID . src

# Install rosdep dependencies
RUN apt-get update \
    && rosdep install -y --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Change entrypoint
RUN sed --in-place \
    's|^source .*|source "/home/ros/overlay/install/setup.bash"|' \
    /ros_entrypoint.sh

# Build overlay
USER $USERNAME
RUN . /home/ros/underlay/install/setup.sh \
    && colcon build

#####################
# Development Image #
#####################
FROM base AS dev

# Install apt packages and ros dependencies manually
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    sudo \
    neovim \
    less \
    usbutils \
    net-tools \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-visual-tools
    # ros-$ROS_DISTRO-ros-gz
    # && rm -rf /var/lib/apt/lists/*

# Add sudo privileges to user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Source underlay in bashrc
USER $USERNAME
RUN echo "source /home/ros/underlay/install/setup.bash" >> ~/.bashrc

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
