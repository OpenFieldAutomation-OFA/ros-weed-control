# teknic_hardware

This package contains the [ros2_control](https://control.ros.org/master/index.html) hardware interface to control Teknic Clearpath SC servo motors with a `SystemInterface`. It was tested on a single CPM-SCSK-2321S-EQNA.

## Motor Setup
The motor should be setup with the ClearView software. The details can be found in the [User Manual](https://teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf). The hardware interface assumes that the motor is tuned.

## Immediate Moves
This package uses the sFoundation software library and a feature called "Immediate Moves" (not in the official documentation) which allows to execute moves immediatly instead of storing them in a queue.

> [!NOTE]
> This will only work with the Clearpath-SC "Advanced" firmware option.

## Homing
TODO

## Hardware Interfaces
The following command interfaces are published:
- `position`
- `velocity`

The following state interfaces are published:
- `position`
- `velocity`
- `effort` (if `peak_torque` specified)

The hardware interfaces can also be listed by starting the controller manager and running the following command.
```
ros2 control list_hardware_interfaces
```

You can use any controller that claims either the position interface or the velocity interface. Claiming both interfaces at the same time is not possible.

## 

## Parameters
Joint:
- `port`: The serial port of the connected SC4-Hub
- `node`: Node number of the motor
- `feed_constant`: Defines the conversion between one revolution of the output shaft to the distance traveled by the linear axis in m/rev. This needs to be set for `prismatic` joints and omitted for `revolute` joints.
- `vel_limit`: Velocity limit in rad/s (without `feed_constant`) or m/s (with `feed_constant`). Used for position moves.
- `acc_limit`: Acceleration limit in rad/s^2 (without `feed_constant`) or m/s^2 (with `feed_constant`). Used for position and velocity moves.
- `peak_torque`: OPTIONAL. Peak torque of the motor in Nm. This is necessary if you want the `effort` state interface.

Note that the acceleration and velocity limits defined here take precedence over any limits of your controller. E. g. if you use MoveIt with a `JointTrajectoryController` the limits in `joint_limits.yaml` need to be the same or smaller to work properly.