"""Launch the Nav2 pydantic-ai command agent."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('nav2_agent')
    default_params_file = PathJoinSubstitution([package_share, 'config', 'agent_params.yaml'])
    default_bt_catalog = PathJoinSubstitution([package_share, 'config', 'bt_catalog.yaml'])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS 2 parameter YAML file for the Nav2 agent.',
    )
    bt_catalog_arg = DeclareLaunchArgument(
        'bt_catalog_path',
        default_value=default_bt_catalog,
        description='Path to the Behavior Tree catalog YAML file.',
    )

    nav2_agent_node = Node(
        package='nav2_agent',
        executable='nav2_agent_node',
        name='nav2_agent_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'bt_catalog_path': LaunchConfiguration('bt_catalog_path')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        bt_catalog_arg,
        nav2_agent_node,
    ])
