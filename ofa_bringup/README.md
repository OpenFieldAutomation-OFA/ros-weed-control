# ofa_bringup

This package contains all launch files of the OFA Weed Control Unit.

To edit the MoveIt 2 configuration:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
Note that the `ofa_moveit_config` folder does not have the exact same structure as the setup assistant would expect and most files have been modified outside of the setup assistant.