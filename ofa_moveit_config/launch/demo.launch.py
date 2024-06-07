from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    launch_arguments = {
        "use_mock_hardware": "true",
    }

    moveit_config = (
        MoveItConfigsBuilder("ofa_robot", package_name="ofa_moveit_config")
        .robot_description(mappings=launch_arguments)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
