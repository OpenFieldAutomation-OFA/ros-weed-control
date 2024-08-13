# ROS distribution
ARG ROS_DISTRO=iron

FROM ros:$ROS_DISTRO-ros-base

# Define user
ARG UID=1000
ARG GID=1000
ARG USERNAME=ofa

# Create user
RUN groupadd --gid $GID $USERNAME \
    && useradd -s /bin/bash --uid $UID --gid $GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Create workspace
WORKDIR /home/$USERNAME/ros2_ws/src

# Clone external dependencies
COPY dependencies.repos .
RUN vcs import  < dependencies.repos

# Install apt packages and ros dependencies manually
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    neovim \
    less \
    python3-pip \
    libpcl-dev \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-ros-perception

USER $USERNAME
RUN echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc

# pip install onnxruntime