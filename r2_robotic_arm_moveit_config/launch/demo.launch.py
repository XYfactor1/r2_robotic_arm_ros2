from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "r2_robotic_arm", package_name="r2_robotic_arm_moveit_config"
        ).to_moveit_configs()
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("r2_robotic_arm_description"), "urdf", "r2_robotic_arm.urdf.xacro"]
            ),
        ]
    )

    moveit_config.robot_description = {"robot_description": robot_description}

    return generate_demo_launch(moveit_config)
