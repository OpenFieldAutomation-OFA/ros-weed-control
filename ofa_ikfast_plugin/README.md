# ofa_ikfast_plugin
This package contains the IKFast plugin for the ofa_robot arm.

## Recreate Plugin
After every change in the robot description, this plugin has to be recreated.

1. In an environment where everything is [setup](../README.md#Setup), generate the URDF file.
    ```bash
    xacro src/ofa_description/urdf/ofa_robot.urdf.xacro > src/ofa_description/urdf/ofa_robot.urdf
    ```
2. On a linux machine that has docker installed, run the script.
    ```bash
    src/ofa_ikfast_plugin/auto_create_ikfast.sh
    ```

Note that the second step can only be done on a `amd64` platform and not directly on the Nvidia Jetson. However, you don't have to install ROS or anything else on the machine to run the script.