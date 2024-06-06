# ofa_ikfast_plugin
This package contains the IKFast plugin for the ofa_robot arm.

## Recreate Plugin
After every change in the robot description, this plugin has to be recreated, so that the kinematics are correctly calculated. Assuming you are developing on the Nvidia Jetson (`arm64`) you need to do the following two steps.

1. On the Jetson, generate the URDF file.
    ```bash
    xacro ~/overlay/src/ofa_description/urdf/ofa_robot.urdf.xacro > ~/overlay/src/ofa_description/urdf/ofa_robot.urdf
    ```
    Then commit and push the repository.
2. On a `linux/amd64` machine that has Docker installed, pull the repository with the new URDF file and run the `auto_update_ikfast.sh` script.

Note that the second step cannot be executed on a `arm64` platform. However, you don't need to install ROS to run `auto_update_ikfast.sh`.