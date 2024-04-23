# BASE STAGE ------------------------------------------------------
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base AS base

# UID and GID should be the same as local user when building dev
ARG UID=1000
ARG GID=1000
ARG USERNAME=ros

# Create non-root user
RUN groupadd --gid $GID $USERNAME \
    && useradd -s /bin/bash --uid $UID --gid $GID -m $USERNAME

# Create ROS workspace
RUN su $USERNAME -c "mkdir /home/$USERNAME/ros2_ws"
WORKDIR /home/$USERNAME/ros2_ws

# Copy source code
COPY --chown=$UID:$GID . src

# Install rosdep dependencies
RUN apt-get update \
    && rosdep install -y --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# PRODUCTION STAGE ------------------------------------------------
FROM base AS prod

USER $USERNAME

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

# TODO: entrypoint script for launching main program

# DEVELOPMENT STAGE -----------------------------------------------
FROM base AS dev

# Install apt packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    sudo \
    neovim \
    ros-$ROS_DISTRO-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Add sudo privileges to user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# Add ROS setup file to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

