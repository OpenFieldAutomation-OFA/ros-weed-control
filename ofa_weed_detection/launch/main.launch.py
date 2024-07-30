from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = PathJoinSubstitution(
        [
            FindPackageShare("ofa_weed_detection"),
            "config",
            "parameters.yaml"
         ]
    )

    # Main node
    weed_detection_node = Node(
        namespace="ofa",
        name="weed_detection",
        package="ofa_weed_detection",
        executable="process_image",
        output="screen",
        parameters=[
            parameters
        ],
    )

    return LaunchDescription([weed_detection_node])