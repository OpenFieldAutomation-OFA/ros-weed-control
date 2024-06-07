# ofa_ikfast_plugin
This package contains the IKFast plugin for the ofa_robot arm.

## Update Plugin
After every change in the URDF file this plugin has to be updated. Otherwise, the inverse kinematics are not calculated correctly.

If you are developing on a `arm64` platform (e. g. Nvidia Jetson) the procedure has to be split into two steps.

1. On the computer with ROS installed, generate the URDF file.
    ```bash
    xacro ofa_robot.urdf.xacro > ofa_robot.urdf
    ```
    Then commit and push the repository.
2. On a computer with `amd64` architecture, pull the repository with the new URDF file and run the script. This cannot be executed on a `arm64` platform.
    ```bash
    bash auto_update_ikfast.sh
    ```
    Then commit and push the changes.

For the second step you only need to have Docker and Python installed, ROS is not needed.