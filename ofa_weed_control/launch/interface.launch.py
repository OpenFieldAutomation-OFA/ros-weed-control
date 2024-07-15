from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ofa_robot", package_name="ofa_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    positions = PathJoinSubstitution(
        [
            FindPackageShare("ofa_weed_control"),
            "config",
            "positions.yaml"
         ]
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
            positions
        ],
    )

    return LaunchDescription([move_group_demo])
