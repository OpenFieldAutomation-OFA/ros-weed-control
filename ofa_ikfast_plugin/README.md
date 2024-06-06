# ofa_ikfast_plugin
This package contains the IKFast plugin for the ofa_robot arm.

## Recreate Plugin
After every change in the robot description, this plugin has to be recreated, so that the kinematics are correctly calculated. Assuming you are developing on the Nvidia Jetson (`arm64`) and you have [setup](../README.md#Setup) everything, this involves doing the following three steps.

1. On the Jetson, generate the URDF file.
    ```bash
    xacro ~/overlay/src/ofa_description/urdf/ofa_robot.urdf.xacro > ~/overlay/src/ofa_ikfast_plugin/tmp/ofa_robot.urdf
    ```
    The `ofa_robo.urdf` file is needed for the next step.
2. On a `linux/amd64` machine that has Docker installed, run the `auto_create_ikfast.sh` script.
     ```bash
    ./create_ikfast_solver.sh tmp/ofa_robot.urdf tmp/solver.cpp
    ```
    The `solver.cpp` file is needed for the next step.
3. Back on the Jetson, generate the plugin. 
     ```bash
    cd ~/overlay/src
    ros2 run moveit_kinematics create_ikfast_moveit_plugin.py\
        --moveit_config_pkg=ofa_moveit_config \
        ofa_robot\
        arm\
        ofa_ikfast_plugin\
        base_link\
        eef_link\
        ofa_ikfast_plugin/tmp/solver.cpp
    rm ofa_ikfast_plugin/update_ikfast_plugin.sh
    ```

Note that the second step (which uses the OpenRAVE Docker image) cannot be executed on a `arm64` platform. This means you have to transmit the intermediate files between the steps (e. g. using Git).
