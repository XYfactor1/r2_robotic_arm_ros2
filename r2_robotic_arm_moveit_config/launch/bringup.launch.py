import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    can_interface = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface name (e.g., can0)",
    )

    can_fd = DeclareLaunchArgument(
        "can_fd",
        default_value="true",
        description="Enable CAN-FD if true",
    )

    kp1 = DeclareLaunchArgument("kp1", default_value="70.0")
    kp2 = DeclareLaunchArgument("kp2", default_value="70.0")
    kp3 = DeclareLaunchArgument("kp3", default_value="70.0")
    kp4 = DeclareLaunchArgument("kp4", default_value="60.0")
    kp5 = DeclareLaunchArgument("kp5", default_value="10.0")
    kd1 = DeclareLaunchArgument("kd1", default_value="2.75")
    kd2 = DeclareLaunchArgument("kd2", default_value="2.5")
    kd3 = DeclareLaunchArgument("kd3", default_value="2.0")
    kd4 = DeclareLaunchArgument("kd4", default_value="2.0")
    kd5 = DeclareLaunchArgument("kd5", default_value="0.7")
    use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock ros2_control hardware if true",
    )
    joint3_lead_m = DeclareLaunchArgument(
        "joint3_lead_m",
        default_value="0.012",
        description="joint_3 screw lead in meters per screw revolution",
    )
    joint3_gear_ratio = DeclareLaunchArgument(
        "joint3_gear_ratio",
        default_value="1.0",
        description="joint_3 gear ratio (motor_turns / screw_turns)",
    )

    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz if true",
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
    )

    moveit_share = get_package_share_directory("r2_robotic_arm_moveit_config")
    rviz_default = os.path.join(moveit_share, "config", "moveit.rviz")

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_default,
        description="RViz config file",
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("r2_robotic_arm_description"), "urdf", "r2_robotic_arm.urdf.xacro"]
            ),
            " ",
            "can_interface:=",
            LaunchConfiguration("can_interface"),
            " ",
            "can_fd:=",
            LaunchConfiguration("can_fd"),
            " ",
            "kp1:=",
            LaunchConfiguration("kp1"),
            " ",
            "kp2:=",
            LaunchConfiguration("kp2"),
            " ",
            "kp3:=",
            LaunchConfiguration("kp3"),
            " ",
            "kp4:=",
            LaunchConfiguration("kp4"),
            " ",
            "kp5:=",
            LaunchConfiguration("kp5"),
            " ",
            "kd1:=",
            LaunchConfiguration("kd1"),
            " ",
            "kd2:=",
            LaunchConfiguration("kd2"),
            " ",
            "kd3:=",
            LaunchConfiguration("kd3"),
            " ",
            "kd4:=",
            LaunchConfiguration("kd4"),
            " ",
            "kd5:=",
            LaunchConfiguration("kd5"),
            " ",
            "use_mock_hardware:=",
            LaunchConfiguration("use_mock_hardware"),
            " ",
            "joint3_lead_m:=",
            LaunchConfiguration("joint3_lead_m"),
            " ",
            "joint3_gear_ratio:=",
            LaunchConfiguration("joint3_gear_ratio"),
            " ",
        ]
    )

    moveit_config = (
        MoveItConfigsBuilder("r2_robotic_arm", package_name="r2_robotic_arm_moveit_config")
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": robot_description}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    ros2_controllers_path = os.path.join(moveit_share, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[moveit_config.to_dict()],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription(
        [
            can_interface,
            can_fd,
            kp1,
            kp2,
            kp3,
            kp4,
            kp5,
            kd1,
            kd2,
            kd3,
            kd4,
            kd5,
            use_mock_hardware,
            joint3_lead_m,
            joint3_gear_ratio,
            use_rviz,
            use_sim_time,
            rviz_config,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            move_group,
            rviz2,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=3.0, actions=[arm_controller_spawner]),
        ]
    )
