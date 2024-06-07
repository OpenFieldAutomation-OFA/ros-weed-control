from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ofa_robot", package_name="ofa_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="arm_moveit",
        package="ofa_weed_control",
        executable="arm_moveit",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])
