# teknic_hardware

This package contains the [ros2_control](https://control.ros.org/master/index.html) hardware interface to control Teknic Clearpath SC servo motors with a `SystemInterface`. It was tested on a single CPM-SCSK-2321S-EQNA.

## Motor Setup
The motor should be setup with the ClearView software. The details can be found in the [User Manual](https://teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf). The hardware interface assumes that the motor is tuned.

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
- `effort`

The hardware interfaces can also be listed by starting the controller manager and running the following command.
```
ros2 control list_hardware_interfaces
```

You can use any controller that claims either the position interface or the velocity interface. Claiming both interfaces at the same time is not possible.

## Parameters
Joint:
- `port`: The serial port of the connected SC4-Hub
- `node`: Node number of the motor
- `acc_limit`: OPTIONAL. Acceleration limit in RPM/s. Used for position and velocity moves. If not set limit is left unchanged.
- `vel_limit`: OPTIONAL. Velocity limit in RPM. Used for position moves. If not set limit is left unchanged.
- `feed_constant`: OPTIONAL. Defines the conversion between one revolution of the output shaft to the distance traveled by the linear axis in mm/rev. This should be set for `prismatic` joints.

