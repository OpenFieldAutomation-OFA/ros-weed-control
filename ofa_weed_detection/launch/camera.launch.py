from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_arguments = {
        "enable_colored_point_cloud": 'false',
        "enable_d2c_viewer": 'false',
        "enable_color": 'true',
        "enable_depth": 'true',
        "enable_ir": 'true',
        "enable_accel": 'false',
        "enable_gyro": 'false',
        "enable_laser": 'true',
        "depth_registration": 'true',
        "color_width": '1920',
        "color_height": '1080',
        "color_fps": '5',
        "ir_width": '1024',
        "ir_height": '1024',
        "ir_fps": '5',
        "depth_width": '1024',
        "depth_height": '1024',
        "depth_fps": '5',
    }
    launch_file = PathJoinSubstitution(
        [
            FindPackageShare("orbbec_camera"),
            "launch",
            "femto_bolt.launch.py"
        ]
    )
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=launch_arguments.items()
    )
    # camera_node = Node(
    #     package="orbbec_camera",
    #     executable="orbbec_camera_node",
    #     name="ob_camera_node",
    #     namespace="camera",
    #     parameters=[camera_config],
    #     output="screen",
    # )
    return LaunchDescription(
        [
            camera_node
        ]
    )
