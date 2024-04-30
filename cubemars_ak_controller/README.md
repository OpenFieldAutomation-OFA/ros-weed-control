# cubemars_ak_controller

This package contains a hardware interface to control the CubeMars AK series motors with [ros2_control](https://control.ros.org/master/index.html). It was tested on the AK70-10.

It implements the following hardware interfaces:
...


## Setup
The package assumes that the motor is setup in Servo Mode. Although the MIT Mode is simple to use and [open source](https://github.com/bgkatz/3phase_integrated), it does not have a lot of options for configuring the control loops. The Servo Mode allows setting PI parameters for the speed loop and PID parameters for the position loop.

To adjust the parameters you have to connect the motor with the R-link and use the "Upper Computer" program. All the information can be found on their [Technical Support and Download page](https://www.cubemars.com/article.php?id=261). Example parameters can be found in [`cubemars_params`](../cubemars_params/).