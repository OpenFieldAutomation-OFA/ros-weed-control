# ofa_bringup

This package contains all launch files of the OFA Weed Control Unit.

## `display.launch.py`
This launches RViz and displays the URDF description of the robot stored in [`ofa_robot_description.urdf.xacro`](../ofa_moveit_config/urdf/ofa_robot_description.urdf.xacro)
This will launch the camera node of the official [Orbbec ROS 2 SDK](https://github.com/orbbec/OrbbecSDK_ROS2) package. This is useful for checking if the camera is working properly. The launch parameters are described on the GitHub.

## MoveIt 2 Setup Assistant
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
This will launch the Moveit 2 Setup Assistant which can be used to create or change a MoveIt configuration package. Note that the `ofa_moveit_config` folder does not have the exact same structure as the setup assistant would expect and most files have been modified from the original configuration. The setup assistant should only be used for updating the SRDF file after the URDF file has been modified.