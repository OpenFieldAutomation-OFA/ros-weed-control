# ofa_bringup

This package contains all launch files of the OFA weed control unit.

## `display.launch.py`
This launches RViz and displays the URDF description of the robot stored in [`ofa_robot.urdf.xacro`](../ofa_moveit_config/config/ofa_robot.urdf.xacro). The joint state publisher GUI can be used to change the joint values.

## `read_only.launch.py`
This launch file starts the hardware interfaces of the motors and prints their current position. It should be used to check if the real position of the robot arm aligns with the robot state before running the main launch file.

## `main.launch.py`
This is the main launch file and it runs all nodes of the weed control unit. It takes a few seconds for all nodes to start because the neural network has to be loaded into memory.

After all nodes are started, you can send an action goal to the `weed_control` node in a new terminal to start the weed detection process. The action goal is returned after the entire workspace is treated.

```bash
ros2 action send_goal /weed_control ofa_interfaces/action/WeedControl "{}" --feedback
```

The parameters of the weed detection are stored in [`weed_parameters.yaml`](config/weed_parameters.yaml).

## `moveit.launch.py`
This file launches the MoveIt 2 plugin in RViz which can be used to manually set the positions of the robot arm.

## `use_mock_hardware` Launch Argument
The `main.launch.py` and `moveit.launch.py` launch files have a launch argument called `use_mock_hardware`. If set to `true` (default) the real hardware of the robot is not used. Instead [mock components](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) simulate the motor positions and [mock images](../ofa_main/images/) are used to replace the camera. This is useful for debugging and developing.

To run the program with the hardware enabled use the following command.
```bash
ros2 launch ofa_bringup main.launch.py use_mock_hardware:=false
```