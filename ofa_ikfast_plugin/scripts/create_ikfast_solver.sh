#!/bin/bash

# This is an adapted version of the script at
# moveit2/moveit_kinematics/ikfast_kinematics_plugin/scripts/auto_create_ikfast_moveit_plugin.sh
# It takes a urdf description and generates the ikfast cpp solver.

# This script pulls the personalrobotics/ros-openrave image.
# Therefore it only runs on linux/amd64 architectures.
# It will also not work when called inside a docker container.

set -e  # fail on error

INPUT=${1:-"`dirname $0`"/../tmp/ofa_robot.urdf}
OUTPUT=${2:-"`dirname $0`"/../tmp/solver.cpp}
BASE_LINK=base_link
EEF_LINK=eef_link
IK_TYPE=Translation3D
PKG_NAME=ofa_ikfast_plugin
ROBOT_NAME=ofa_robot

# Create a temporary directory to operate in
TMP_DIR=$(mktemp -d --tmpdir ikfast.XXXXXX)
trap "rm -rf $TMP_DIR" EXIT

# build docker image
echo "Building docker image"
cat <<EOF > $TMP_DIR/Dockerfile
FROM personalrobotics/ros-openrave
# Update ROS keys (https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454, https://github.com/osrf/docker_images/issues/697)
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
apt-get update && \
apt-get install -y --force-yes --no-install-recommends python-pip build-essential liblapack-dev ros-indigo-collada-urdf && \
apt-get clean && rm -rf /var/lib/apt/lists/*
# enforce a specific version of sympy, which is known to work with OpenRave
RUN pip install git+https://github.com/sympy/sympy.git@sympy-0.7.1
EOF
docker build -t fixed-openrave $TMP_DIR
echo "Successfully built docker image."

# create dae file
cp "$INPUT" "$TMP_DIR/robot.urdf"
docker run --rm --user $(id -u):$(id -g) -v $TMP_DIR:/input --workdir /input -e HOME=/input \
    fixed-openrave:latest rosrun collada_urdf urdf_to_collada robot.urdf robot.dae

# create solver
cat <<EOF > $TMP_DIR/wrapper.xml
<robot file="robot.dae">
<Manipulator name="$ROBOT_NAME">
<base>$BASE_LINK</base>
<effector>$EEF_LINK</effector>
</Manipulator>
</robot>
EOF
cmd="openrave0.9.py --database inversekinematics --robot=/input/wrapper.xml --iktype=$IK_TYPE --iktests=1000"
echo "Running $cmd"
docker run --rm --user $(id -u):$(id -g) \
    -v $TMP_DIR:/input --workdir /input -e HOME=/input \
    fixed-openrave:latest $cmd
CPP_FILE=$(ls -1 $TMP_DIR/.openrave/*/*.cpp 2> /dev/null)
if [ -n "$CPP_FILE" ] ; then
    echo "Created $CPP_FILE"
    cp "$CPP_FILE" "$OUTPUT"
    echo "Saved $OUTPUT"
else
    echo "Failed to create ikfast solver"
fi