import os
import socket

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths


def _allocate_gazebo_master_uri():
    """Reserve an ephemeral localhost port and use it as the Gazebo master URI."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    return f'http://127.0.0.1:{port}'


def _configure_scene(context, fishbot_description_dir, default_world_path):
    scene = launch.substitutions.LaunchConfiguration('scene').perform(context)
    requested_world = launch.substitutions.LaunchConfiguration('world').perform(context)

    scene_worlds = {
        'narrow': fishbot_description_dir + '/world/slam_narrow_corridor.world',
        's_curve': fishbot_description_dir + '/world/slam_s_curve_corridor.world',
        'l_turn': fishbot_description_dir + '/world/slam_l_turn_corridor.world',
    }

    # Keep explicit world:=... overrides working. Scene selection only replaces
    # the default world path when the user hasn't provided a custom one.
    resolved_world = requested_world
    if scene in scene_worlds and requested_world == default_world_path:
        resolved_world = scene_worlds[scene]

    return [
        launch.actions.SetLaunchConfiguration('resolved_world', resolved_world),
        launch.actions.LogInfo(msg=['Using scene: ', scene]),
        launch.actions.LogInfo(msg=['Using world file: ', resolved_world]),
    ]


def _gazebo_client_env():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in os.environ and os.environ['GAZEBO_MODEL_PATH']:
        model_path = model_path + os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in os.environ and os.environ['GAZEBO_PLUGIN_PATH']:
        plugin_path = plugin_path + os.pathsep + os.environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in os.environ and os.environ['GAZEBO_RESOURCE_PATH']:
        media_path = media_path + os.pathsep + os.environ['GAZEBO_RESOURCE_PATH']

    return {
        'GAZEBO_MODEL_PATH': model_path,
        'GAZEBO_PLUGIN_PATH': plugin_path,
        'GAZEBO_RESOURCE_PATH': media_path,
    }


def generate_launch_description():
    robot_name_in_model = "fishbot"
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    default_model_path = fishbot_description_dir + '/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = fishbot_description_dir + '/world/custom_room.world'
    default_controller_config = fishbot_description_dir + '/config/fishbot_ros2_controller.yaml'
    default_gazebo_master_uri = _allocate_gazebo_master_uri()

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    action_declare_arg_use_sim_time = launch.actions.DeclareLaunchArgument(
        name='use_sim_time', default_value='true',
        description='是否使用仿真时间 /clock')
    action_declare_arg_scene = launch.actions.DeclareLaunchArgument(
        name='scene', default_value='custom',
        description='预设场景名称: custom | narrow | s_curve | l_turn')
    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world', default_value=str(default_world_path),
        description='Gazebo world 的绝对路径')
    action_declare_arg_gazebo_master_uri = launch.actions.DeclareLaunchArgument(
        name='gazebo_master_uri', default_value=str(default_gazebo_master_uri),
        description='Gazebo master URI，默认自动分配独立端口')
    action_declare_arg_controller_config = launch.actions.DeclareLaunchArgument(
        name='controller_config', default_value=str(default_controller_config),
        description='ros2_control 控制器参数文件路径')
    action_declare_arg_gui = launch.actions.DeclareLaunchArgument(
        name='use_gui_client', default_value='true',
        description='是否启动 Gazebo GUI 客户端')
    action_declare_arg_gui_start_delay = launch.actions.DeclareLaunchArgument(
        name='gui_start_delay', default_value='2.0',
        description='gzclient 延迟启动秒数，避免在 gzserver 初始化前抢跑')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    resolved_world = launch.substitutions.LaunchConfiguration('resolved_world')
    gazebo_master_uri = launch.substitutions.LaunchConfiguration('gazebo_master_uri')
    controller_config = launch.substitutions.LaunchConfiguration('controller_config')
    use_gui_client = launch.substitutions.LaunchConfiguration('use_gui_client')
    gui_start_delay = launch.substitutions.LaunchConfiguration('gui_start_delay')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command([
            'xacro ',
            launch.substitutions.LaunchConfiguration('model'),
            ' controller_config:=',
            controller_config,
        ]),
        value_type=str)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py',
        ]),
        launch_arguments={
            'world': resolved_world,
            'verbose': 'true',
            'gui': 'false',
            'server': 'true',
        }.items()
    )

    launch_gzclient = launch.actions.TimerAction(
        period=gui_start_delay,
        condition=launch.conditions.IfCondition(use_gui_client),
        actions=[
            launch.actions.LogInfo(
                msg=['Starting gzclient after ', gui_start_delay, 's delay'],
            ),
            launch.actions.ExecuteProcess(
                cmd=[
                    'gzclient',
                    '--gui-client-plugin=libgazebo_ros_eol_gui.so',
                    '--verbose',
                ],
                name='gzclient',
                output='screen',
                additional_env=_gazebo_client_env(),
                respawn=True,
                respawn_delay=2.0,
            ),
        ],
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_name_in_model],
        parameters=[{'use_sim_time': use_sim_time}])

    load_joint_state_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fishbot_joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    load_fishbot_diff_drive_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fishbot_diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen')

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_arg_use_sim_time,
        action_declare_arg_scene,
        action_declare_arg_world_path,
        action_declare_arg_gazebo_master_uri,
        action_declare_arg_controller_config,
        action_declare_arg_gui,
        action_declare_arg_gui_start_delay,
        launch.actions.OpaqueFunction(
            function=lambda context: _configure_scene(
                context,
                fishbot_description_dir,
                default_world_path,
            )
        ),
        launch.actions.SetEnvironmentVariable(
            name='GAZEBO_MASTER_URI',
            value=gazebo_master_uri,
        ),
        launch.actions.LogInfo(
            msg=['Using GAZEBO_MASTER_URI: ', gazebo_master_uri],
        ),
        robot_state_publisher_node,
        launch_gazebo,
        launch_gzclient,
        spawn_entity_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],)
            ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller],)
            ),
    ])
