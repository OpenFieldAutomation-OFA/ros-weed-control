# cubemars_hardware

This package contains the [ros2_control](https://control.ros.org/master/index.html) hardware interface to control the CubeMars AK series motors with a  `SystemInterface`. It was tested on the AK70-10.

The following hardware interfaces are published:
- command interfaces:
  - acceleration
  - effort
  - position
  - velocity
- state interfaces:
  - effort
  - position
  - temperature
  - velocity

This list can also be seen by running `ros2 control list_hardware_interfaces` after having started the controller manager.

The controller has to claim one of the following combinations of command interfaces:
- `effort` (Current Loop Mode)
- `speed` (Speed Loop Mode)
- `position` (Position Loop Mode)
- `position`, `speed` and `acceleration` (Position-Speed Loop Mode)


## Setup
The package assumes that the motor is setup in Servo Mode. Although the MIT Mode is simple to use and [open source](https://github.com/bgkatz/3phase_integrated), it does not have a lot of options for configuring the control loops. The Servo Mode allows setting PI parameters for the speed loop and PID parameters for the position loop.

To adjust the parameters you have to connect the motor with the R-link and use the "Upper Computer" program. All the information can be found on their [Technical Support and Download page](https://www.cubemars.com/article.php?id=261). Example parameters can be found in [`cubemars_params`](../.params/).

Make sure that you enable "Send status over CAN" (called `send_can_status` in AppParams). Also, the upload frequency of the actuators should be at least as fast as the update rate of the controller manager. If no CAN message is received during one update loop you will get a warning in the `read` function.