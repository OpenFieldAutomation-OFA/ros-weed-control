# ROS distribution
ARG ROS_DISTRO=iron

FROM osrf/ros:$ROS_DISTRO-desktop

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

USER $USERNAME

# Create workspace
WORKDIR /home/$USERNAME/ros2_ws/src

# Clone external dependencies
COPY dependencies.repos .
RUN vcs import < dependencies.repos && rm dependencies.repos

# Install apt packages and ros dependencies manually
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
    neovim \
    less \
    wget \
    python3-pip \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-moveit \
    python3-open3d \
    python3-sklearn

RUN pip install onnxruntime numpy==1.26.4

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc