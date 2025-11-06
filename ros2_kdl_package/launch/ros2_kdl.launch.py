from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package path
    pkg_path = get_package_share_directory('ros2_kdl_package')
    # Params: containing the robot description
    params = os.path.join(pkg_path, 'config', 'param.yaml')


    #### Nodes ####

    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        output='screen',
        parameters=[params],
    )


    # List of Arguments and Nodes
    return LaunchDescription([
        ros2_kdl_node
    ])