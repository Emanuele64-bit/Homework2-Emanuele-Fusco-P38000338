from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Package path
    pkg_path = get_package_share_directory('ros2_kdl_package')
    # Params: containing the robot description
    params = os.path.join(pkg_path, 'config', 'param.yaml')

    #### Arguments ####

    controller_arg = DeclareLaunchArgument(
        name='ctrl',
        description = 'Select: "velocity_ctrl" for velocity controller; "velocity_ctrl_null" for velocity controller in the null space',
        default_value='velocity_ctrl',
    )
    ctrl = LaunchConfiguration("ctrl")

    cmd_interface = DeclareLaunchArgument(
        name='cmd_interface',
        description = 'Select type of controller',
        default_value='velocity',
    )
    cmd = LaunchConfiguration("cmd_interface")

    node_to_start = DeclareLaunchArgument(
        name='node',
        description = 'Select the node to start',
        default_value='server',
    )
    node = LaunchConfiguration("node")

    #### Nodes ####

    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        output='screen',
        parameters=[params, {'cmd_interface': cmd}, {"ctrl" : ctrl}],
        condition=IfCondition(
            PythonExpression([
                "'", node, "' == 'server'",
            ])
        )
    )

    ros2_kdl_client_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_client_node',
        output='screen',
        parameters=[params],
        condition=IfCondition(PythonExpression(["'", node, "' == 'client'"]))
    )

    # List of Arguments and Nodes
    return LaunchDescription([
        node_to_start,
        cmd_interface,
        controller_arg,
        ros2_kdl_node,
        ros2_kdl_client_node,
    ])