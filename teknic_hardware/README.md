# teknic_hardware

This package contains the [ros2_control](https://control.ros.org/master/index.html) hardware interface to control Teknic Clearpath SC servo motors with a `SystemInterface`. It was tested on a single CPM-SCSK-2321S-EQNA.

## Motor Setup
The motor should be setup with the ClearView software. The details can be found in the [User Manual](https://teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf). The hardware interface assumes that the motor is tuned.

## sFoundation
This package uses the sFoundation software library. This library does not allow direct access to the reference position / velocity. Instead we have to create a "move" with a predefined maximum acceleration (and velocity).

In the implementation of this interface we only change the target command when the previous move is done. This means the maximum acceleration and velocity parameters should be set high if your controller has a high update frequency (much higher than the expected task acceleration / velocity). Otherwise there will be a delay between the commanded position and the reference position of the Clearpath motor.

Furthremore, it might make sense to disable the tracking error limit when using high acceleration and velocity values.

This is not a clean solution but rather a hack because we cannot access the reference position / velocity directly.

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