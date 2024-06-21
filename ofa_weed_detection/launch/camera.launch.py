from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config = PathJoinSubstitution(
        [
            FindPackageShare("ofa_weed_detection"),
            "config",
            "camera.yaml"
         ]
    )
    camera_node = Node(
        package="orbbec_camera",
        executable="orbbec_camera_node",
        name="ob_camera_node",
        namespace="camera",
        parameters=[camera_config],
        output="screen",
    )
    return LaunchDescription(
        [
            camera_node
        ]
    )
