# ofa_moveit_config
This package contains the MoveIt configuration for the `ofa_robot`.

## Update SRDF File
To update the SRDF file you first need to launch the MoveIt Setup Assistant.

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Select **Edit Existing MoveIt Configuration Package** and choose the `ofa_moveit_config` folder. Then go to the **Self-Collisions** tab and click on **Generate Collision Matrix**. Navigate to the **Configuration Files** tab and deselect everything except the `config/ofa_robot.srdf` file. Finally, click on **Generate Package**.

The other files should not be changed with the MoveIt Setup Assistant because they have been modified externally.
