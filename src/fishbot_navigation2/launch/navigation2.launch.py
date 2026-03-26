import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml


def _configure_scene(context, fishbot_navigation2_dir, default_map_yaml):
    scene = LaunchConfiguration('scene').perform(context)
    requested_map = LaunchConfiguration('map').perform(context)

    scene_maps = {
        'narrow': os.path.join(fishbot_navigation2_dir, 'maps', 'slam_narrow_corridor.yaml'),
        's_curve': os.path.join(fishbot_navigation2_dir, 'maps', 'slam_s_curve_corridor.yaml'),
        'l_turn': os.path.join(fishbot_navigation2_dir, 'maps', 'slam_l_turn_corridor.yaml'),
    }

    resolved_map = requested_map
    if scene in scene_maps and requested_map == default_map_yaml:
        resolved_map = scene_maps[scene]

    return [
        launch.actions.SetLaunchConfiguration('resolved_map', resolved_map),
        launch.actions.LogInfo(msg=['Using scene: ', scene]),
        launch.actions.LogInfo(msg=['Using map file: ', resolved_map]),
    ]


def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    default_map_yaml = os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml')
    default_nav2_params = os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
    default_ekf_params = os.path.join(fishbot_navigation2_dir, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')
    resolved_map = LaunchConfiguration('resolved_map')
    nav2_param_path = LaunchConfiguration('params_file')
    ekf_param_path = LaunchConfiguration('ekf_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    odom_topic = PythonExpression([
        "'/odometry/filtered' if '",
        use_ekf,
        "' == 'true' else '/odom'",
    ])

    configured_params = RewrittenYaml(
        source_file=nav2_param_path,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
            'yaml_filename': resolved_map,
        },
        convert_types=True,
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Launch robot_localization EKF and switch Nav2 to filtered odometry'),
        launch.actions.DeclareLaunchArgument(
            'scene',
            default_value='custom',
            description='Preset map scene: custom | narrow | s_curve | l_turn'),
        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 param file to load'),
        launch.actions.DeclareLaunchArgument(
            'ekf_params_file',
            default_value=default_ekf_params,
            description='Full path to EKF param file to load'),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz if true'),
        launch.actions.OpaqueFunction(
            function=lambda context: _configure_scene(
                context,
                fishbot_navigation2_dir,
                default_map_yaml,
            )
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(use_ekf),
            parameters=[ekf_param_path, {'use_sim_time': use_sim_time}],
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': resolved_map,
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
            }.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz)),
    ])
