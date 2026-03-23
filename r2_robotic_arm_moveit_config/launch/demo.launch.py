import os
import xacro
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition

def generate_robot_description(context: LaunchContext, description_package, description_file,
                                use_fake_hardware, can_interface):

    # Substitute launch configuration values
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    can_interface_str = context.perform_substitution(can_interface)

    # Build xacro file path
    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", description_file_str
    )

    # Process xacro with required arguments
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "use_fake_hardware": use_fake_hardware_str,
            "can_interface": can_interface_str,
            "ros2_control": "true",
        }
    ).toprettyxml(indent="  ")

    return robot_description

def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                         use_fake_hardware, controllers_file, can_interface):
    """Spawn both robot state publisher and control nodes with shared robot description."""

    # Generate robot description once
    robot_description = generate_robot_description(
        context, description_package, description_file, use_fake_hardware, can_interface)

    # Get controllers file path
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]

def generate_launch_description():
    """Generate launch description for r2_robotic_arm unimanual configuration."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package", 
            default_value="r2_robotic_arm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file", 
            default_value="r2_robotic_arm.xacro",
            description="URDF/XACRO description file with the robot.",
        ),        
        DeclareLaunchArgument(
            "use_fake_hardware", 
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "can_interface", 
            default_value="can1",
            description="CAN interface to use.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package", 
            default_value="r2_robotic_arm_moveit_config",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "controllers_file", 
            default_value="ros2_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
            ),
        DeclareLaunchArgument(
            "use_rviz", 
            default_value="true", 
            description="Start RViz2 automatically with this launch file."),
    ]        

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    # robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    can_interface = LaunchConfiguration("can_interface")
    # Configuration file paths
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         controllers_file]
    )

    # Robot nodes spawner (both state publisher and control)
    robot_nodes_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file,
              use_fake_hardware, controllers_file, can_interface],
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    r2_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r2_arm_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_arm = TimerAction(period=3.0, actions=[r2_arm_spawner])

    moveit_config = MoveItConfigsBuilder(
        "r2_robotic_arm", package_name="r2_robotic_arm_moveit_config"
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    rviz_config_file = os.path.join(
            get_package_share_directory("r2_robotic_arm_moveit_config"),
            "config", "moveit.rviz"
        )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_params],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_func,
            delayed_jsb,
            delayed_arm,
            run_move_group_node,
            rviz_node,
        ]
    )
