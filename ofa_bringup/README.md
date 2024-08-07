# ofa_bringup

This package contains all launch files of the OFA Weed Control Unit.

## Orbbec ROS 2
```bash
ros2 launch orbbec_camera femto_bolt.launch.py
```
This will launch the camera node of the official [Orbbec ROS 2 SDK](https://github.com/orbbec/OrbbecSDK_ROS2) package. This is useful for checking if the camera is working properly. The launch parameters are described on the GitHub.

## MoveIt 2 Setup Assistant
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
This will launch the Moveit 2 Setup Assistant which can be used to create the MoveIt config. Note that the `ofa_moveit_config` folder does not have the exact same structure as the setup assistant would expect and most files have been modified outside of the setup assistant. The setup assistant should only be used for generating and saving the SRDF file (after he URDF file has been modified) to update the self-collisions.