from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    model_path = PathJoinSubstitution(['urdf', 'robot2.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([FindPackageShare('ofa_description'), 'rviz', 'urdf.rviz'])
    
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'ofa_description',
            'urdf_package_path': model_path,
            'rviz_config': rviz_config_path}.items()
    ))

    return ld