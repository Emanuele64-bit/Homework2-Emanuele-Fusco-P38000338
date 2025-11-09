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

    # aruco_path = get_package_share_directory('iiwa_description')
    # aruco_params = os.path.join(aruco_path, 'gazebo/world', 'empty.world')

    # models_path = PathJoinSubstitution([
    # FindPackageShare('ros2_iiwa'),
    # 'iiwa_description', 'gazebo', 'models'
    # ])

    # SetEnvironmentVariable(
    # name='GZ_SIM_RESOURCE_PATH',
    # value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''), ':', models_path]
    # )

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

    empty_pkg_path = get_package_share_directory('iiwa_description')
    # Params: containing the robot description
    empty_world_path = os.path.join(empty_pkg_path, 'gazebo', 'worlds', 'empty.world')

    empty_world_launch = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
    launch_arguments={
        'pause': 'false',
        'gz_args': ['-r ', empty_world_path],
    }.items(),
    )

    iiwa_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('iiwa_bringup'), 'launch', 'iiwa.launch.py']),
         launch_arguments={
        #     'gui': LaunchConfiguration('gui'),
            'pause': 'false',
            'gz_args': ['-r ', empty_world_path],
        }.items(),
    )

    aruco_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('aruco_ros'), 'launch', 'single.launch.py'
        ]),
        launch_arguments={
            # parametri necessari per aruco_ros
        }.items(),
    )

    #### Nodes ####

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'iiwa', '-z', '0.0', '-unpause'],
        output='screen',
    )

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
        ros2_kdl_node,
        ros2_kdl_client_node,
        iiwa_launch,
        empty_world_launch,
        urdf_spawner_node,
        aruco_launch
    ])