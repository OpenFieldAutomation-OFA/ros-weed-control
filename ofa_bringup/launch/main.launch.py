from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # Load the robot configuration
    mappings = {
        "use_mock_hardware": use_mock_hardware,
    }
    moveit_config = (
        MoveItConfigsBuilder("ofa_robot", package_name="ofa_moveit_config")
        .robot_description(mappings=mappings)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict()],
    )

    # Launch RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ofa_weed_control"), "rviz", "main.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control Controller Manager
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("ofa_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        output="log",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ofa_robot_controller", "--controller-manager", "/controller_manager"],
    )

    # Main node
    weed_parameters = PathJoinSubstitution(
        [
            FindPackageShare("ofa_bringup"),
            "config",
            "weed_parameters.yaml"
         ]
    )
    weed_main_node = Node(
        name="weed_main",
        package="ofa_main",
        executable="weed_control",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            weed_parameters,
            {"use_mock_hardware": use_mock_hardware}
        ],
    )

    # Detection node
    weed_detection_node = Node(
        name="weed_detection",
        package="ofa_detection",
        executable="cluster_classify",
        output="log",
        parameters=[weed_parameters],
    )
    
    delay_weed_main_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[weed_main_node],
        )
    )

    return LaunchDescription(
        declared_arguments + 
        [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            delay_weed_main_node,
            weed_detection_node,
        ]
    )