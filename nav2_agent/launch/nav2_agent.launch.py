"""Launch nav2_agent_node.

Parameters are read from params_file. The launch arguments below override the value in params_file only when set.
"""

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

OVERRIDES = {
    'bt_catalog_path': 'Behavior Tree catalog YAML file.',
    'locations_path': 'Named locations YAML file, or none to disable locations.',
    'dry_run_nav2': 'Log Nav2 goals without sending them (true or false).',
    'use_sim_time': 'Use simulation time from /clock (true or false).',
    'agent_run_timeout_sec': 'Timeout for one agent run, in seconds.',
}
PATH_PARAMETERS = {'bt_catalog_path', 'locations_path'}


def launch_node(context: LaunchContext) -> list:
    overrides = {}
    for name in OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value if name in PATH_PARAMETERS else yaml.safe_load(value)

    return [
        Node(
            package='nav2_agent',
            executable='nav2_agent_node',
            name='nav2_agent_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), overrides],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution([FindPackageShare('nav2_agent'), 'config', 'agent_params.yaml'])
    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Parameter YAML file.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logger level, for example info or debug.'),
    ]
    arguments += [DeclareLaunchArgument(name, default_value='', description=text) for name, text in OVERRIDES.items()]
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_node)])
