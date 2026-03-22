from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("r2_robotic_arm_description")
    rviz_config_path = os.path.join(pkg_share, "rviz", "r2_robotic_arm.rviz")

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_path,
        description="RViz2 config file",
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_config,
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz2,
        ]
    )
