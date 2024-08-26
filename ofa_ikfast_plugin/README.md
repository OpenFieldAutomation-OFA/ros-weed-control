# ofa_ikfast_plugin
This package contains the IKFast plugin for the ofa_robot arm.

## Update Plugin
After every change in the URDF file this plugin has to be updated. Otherwise, the inverse kinematics are not calculated correctly.

If you are developing on the Jetson (`arm64` platform) the procedure has to be split into two steps. For both steps you need to be in the `ros-weed-control` folder.

1. On the Jetson, generate the URDF file.
    ```bash
    xacro ofa_moveit_config/config/ofa_robot.urdf.xacro > ofa_robot.urdf
    ```
    Commit and push the repository.
2. On a computer with `amd64` architecture, pull the repository with the new URDF file and run the script. This cannot be executed on a `arm64` platform.
    ```bash
    bash ofa_ikfast_plugin/scripts/auto_update_ikfast.sh
    ```
    Commit and push the changes.

For the second step you only need to have Docker and Python installed, ROS is not needed.