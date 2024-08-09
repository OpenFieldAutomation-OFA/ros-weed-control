# credit: https://github.com/sea-bass/turtlebot3_behavior_demos

# ROS distribution
ARG ROS_DISTRO=jazzy

##############
# Base Image #
##############
FROM ros:$ROS_DISTRO-ros-base AS base

# Use default user
ARG USER=ubuntu

# Create underlay workspace
WORKDIR /home/$USER/underlay/src

# Clone external dependencies
COPY dependencies.repos .
RUN vcs import  < dependencies.repos

# Build underlay workspace
WORKDIR /home/$USER/underlay
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update \
    && rosdep install -y --from-paths src --ignore-src \
    && colcon build \
    && rm -rf /var/lib/apt/lists/*

# Create overlay workspace
RUN su $USER -c "mkdir /home/$USER/overlay"
WORKDIR /home/$USER/overlay

# Allow access to serial and video devices
RUN usermod -aG dialout,video $USER

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
    "s|^source .*|source /home/$USER/overlay/install/setup.bash|" \
    /ros_entrypoint.sh

# Build overlay
USER $USER
RUN . /home/$USER/underlay/install/setup.sh \
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
    can-utils \
    gdb \
    liblapack-dev \
    libpcl-dev \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-ros2-controllers-test-nodes \ 
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-moveit-ros-perception \
    ros-$ROS_DISTRO-rqt-joint-trajectory-controller
    # ros-$ROS_DISTRO-ros-gz
    # && rm -rf /var/lib/apt/lists/*

# Add sudo privileges to user
RUN echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER && touch /home/$USER/.sudo_as_admin_successful

# Source underlay in bashrc
USER $USER
RUN echo "source /home/$USER/underlay/install/setup.bash" >> ~/.bashrc

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update