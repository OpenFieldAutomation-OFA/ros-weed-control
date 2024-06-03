# cubemars_hardware

This package contains the [ros2_control](https://control.ros.org/master/index.html) hardware interface to control the CubeMars AK series motors with a  `SystemInterface`. It was tested on the AK70-10.

## Motor Setup
The package assumes that the motor is setup in Servo Mode. Although the MIT Mode is simple to use and [open source](https://github.com/bgkatz/3phase_integrated), it has fewer configuration options than the Servo Mode.

To adjust the parameters you can connect the motor with the R-link and use the "Upper Computer" program. All the informations are on their [Technical Support and Download page](https://www.cubemars.com/article.php?id=261). Example parameters can be found in [`.params`](../.params/).

Make sure that you enable "Send status over CAN" (called `send_can_status` in AppParams). Also, the upload frequency of the actuators should be at least as fast as the update rate of the controller manager. If no CAN message is received during one update loop you will get a warning in the `read` function.

## Set Origin
This hardware interface does NOT set the origin of the actuator on startup or at any other time. It assumes the position it receives over CAN is the correct position. This means you will have to set the origin yourself. You can do that with `can-utils` (replace `XX` with the hexadecimal representation of the CAN ID).
```
cansend can0 000005XX#01
```
Most CubeMars actuators don't have an encoder on the output shaft. Therefore, it is usually a good idea to go to the origin before powering off the motor (this will preserve the absolute position).

## Hardware Interfaces
The following command interfaces are published:
- `position`
- `velocity`
- `acceleration`
- `effort`

The following state interfaces are published:
- `position`
- `velocity`
- `effort`
- `temperature`

The hardware interfaces can also be listed by starting the controller manager and running the following command.
```
ros2 control list_hardware_interfaces
```

You can use any controller that claims one of the following command interfaces. You can only claim one at a time.
- `effort` (Current Loop Mode)
- `velocity` (Speed Loop Mode)
- `position` (Position Loop Mode)

Note that the Position-Speed Loop Mode is not implemented. This is done on purpose, as that mode expects a constant value for speed and acceleration, instead of feedforward values.

## Parameters
Hardware:
- `can_interface`: name of the Linux CAN interface, e. g. `can0`

Joint:
- `can_id`: CAN ID of the actuator
- `pole_pairs`: Pole pairs. Used for unit conversion
- `gear_ratio`: Gear ratio. Used for unit conversion
- `kt`: Torque constent. Used to convert current to torque

