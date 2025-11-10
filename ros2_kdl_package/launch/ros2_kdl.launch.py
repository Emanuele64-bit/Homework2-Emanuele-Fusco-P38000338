from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
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

    cmd_interface = DeclareLaunchArgument(
        name='cmd_interface',
        description = 'Select type of controller',
        default_value='velocity_ctrl',
    )
    ctrl = LaunchConfiguration("cmd_interface")

    node_to_start = DeclareLaunchArgument(
        name='node',
        description = 'Select the node to start',
        default_value='server',
    )
    node = LaunchConfiguration("node")

    #### Launch ####

    # empty_world_launch = IncludeLaunchDescription(
    # PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
    # launch_arguments={
    #     'pause': 'true',
    #     'gz_args': ['-r ', 'empty.sdf'],
    # }.items(),
    # )


    # iiwa_launch = IncludeLaunchDescription(
    #     PathJoinSubstitution(
    #         [FindPackageShare('iiwa_bringup'), 'launch', 'iiwa.launch.py']),
    #      launch_arguments={
    #     #     'gui': LaunchConfiguration('gui'),
    #          'pause': 'true',
    #         'gz_args': ['-r ', 'empty.sdf'],
    #     }.items(),
    # )

    # # push robot_description to factory and spawn robot in gazebo
    # urdf_spawner_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     name='urdf_spawner',
    #     arguments=['-topic', '/robot_description', '-entity', 'iiwa', '-z', '0', '-unpause'],
    #     output='screen',
    # )

    #### Nodes ####

    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        output='screen',
        parameters=[{'cmd_interface': ctrl}],
        condition=IfCondition(PythonExpression(["'", node, "' == 'server'"]))
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
        # iiwa_launch,
        # empty_world_launch,
        ros2_kdl_node,
        ros2_kdl_client_node,
        # urdf_spawner_node
    ])