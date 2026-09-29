"""Launch nav2_agent_node."""

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
    dry_run_nav2_arg = DeclareLaunchArgument(
        'dry_run_nav2',
        default_value='false',
        description='Log the Nav2 action goal that would be sent without requiring Nav2 action servers.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from /clock.',
    )
    agent_run_timeout_arg = DeclareLaunchArgument(
        'agent_run_timeout_sec',
        default_value='90.0',
        description='Maximum seconds allowed for one pydantic-ai agent run before failing the command.',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS logger level for nav2_agent_node, for example info or debug.',
    )

    nav2_agent_node = Node(
        package='nav2_agent',
        executable='nav2_agent_node',
        name='nav2_agent_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'bt_catalog_path': LaunchConfiguration('bt_catalog_path'),
                'dry_run_nav2': LaunchConfiguration('dry_run_nav2'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'agent_run_timeout_sec': LaunchConfiguration('agent_run_timeout_sec'),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        params_file_arg,
        bt_catalog_arg,
        dry_run_nav2_arg,
        use_sim_time_arg,
        agent_run_timeout_arg,
        log_level_arg,
        nav2_agent_node,
    ])
