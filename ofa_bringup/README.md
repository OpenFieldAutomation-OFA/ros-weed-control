# ofa_bringup

This package contains all launch files of the OFA Weed Control Unit.

The `main.launch.py` and `moveit.launch.py` launch files have a launch argument called `use_mock_hardware`. If set to `true` (default) the real hardware of the robot is not used. Instead the `ros2_control` [mock components](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) simulate the motor positions and [mock images](../ofa_main/images/) are used to replace the camera. This is useful for debugging and developing.

To run the main program with the hardware enabled use the following command.
```bash
ros2 launch ofa_bringup main.launch.py use_mock_hardware:=false
```

## `display.launch.py`
This launches RViz and displays the URDF description of the robot stored in [`ofa_robot_description.urdf.xacro`](../ofa_moveit_config/urdf/ofa_robot_description.urdf.xacro) and a joint state publisher GUI to change the joint values.

## `read_only.launch.py`
This enables the motors and prints their current position. This can be used to check if the real positions of the motors align with the software robot state. This should always be done before running a launch file with `use_mock_hardware:=false`.

## `main.launch.py`
This is the main launch file that launches all nodes of the weed control unit. The parameters of the weed detection are stored in [`weed_parameters.yaml`](config/weed_parameters.yaml).

It takes a few seconds for all nodes to start because the neural network has to be loaded into memory. After all nodes are started, you can send an action goal to the `weed_control` node in a new terminal to start the weed detection process. This will start the workspace treatment and return the action goal once it is finished.

```bash
ros2 action send_goal /weed_control ofa_interfaces/action/WeedControl "{}" --feedback
```

## `moveit.launch.py`
This file launches the MoveIt 2 plugin in RViz which can be used to manually set the positions of the robot arm.