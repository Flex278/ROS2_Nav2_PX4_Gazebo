"""Полный запуск симуляции UAV.

Собирает воедино:
  1. PX4 SITL (``make px4_sitl gz_x500_depth``) + Gazebo Harmonic (мир lab.world)
  2. MicroXRCEAgent — мост uORB <-> ROS2 (БЕЗ MAVROS)
  3. ros_gz_bridge — сенсоры камеры (image / depth / points) + /clock
  4. uav_offboard — offboard-управление (trajectory_setpoint, MPC, Безье)
  5. uav_sensors — depth -> /scan для Nav2
  6. uav_perception — детектор H-маркера + YOLO (запуск из venv)
  7. Nav2 — навигация (planner / controller / behavior / BT / waypoint)

Запуск:
    bash scripts/run_sim.sh
    # или вручную:
    ros2 launch uav_bringup sim_full.launch.py

Примечания:
  * PX4 запускается отдельным процессом (не Node) из каталога PX4-Autopilot.
  * GZ_SIM_RESOURCE_PATH дополняется (не затирается) путями к нашим моделям
    (h_marker) и моделям PX4.
  * Nav2 поднимается напрямую (planner/controller содержат costmap внутри) —
    это соответствует структуре navigation_launch.py из Jazzy, но без
    опциональных серверов (route/docking/smoother/collision).
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Имена lifecycle-узлов Nav2 (порядок важен для корректного автозапуска).
NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]


def _launch_px4(context, uav_gazebo_share):
    """Запуск PX4 SITL с нашим миром и путём к моделям (h_marker)."""
    if context.launch_configurations['start_px4'].lower() not in ('true', '1'):
        return []

    px4_dir = context.launch_configurations.get('px4_dir') or os.environ.get(
        'PX4_AUTOPILOT_DIR', os.path.join(os.getcwd(), 'PX4-Autopilot'))
    # Модель PX4 для симуляции Gazebo: gz_x500_depth добавляет камеру
    # OakD-Lite (RGB + depth + points), необходимую для Nav2 и CV.
    px4_model = context.launch_configurations.get('px4_model', 'gz_x500_depth')
    world = context.launch_configurations.get('world', 'lab.world')
    world_path = os.path.join(uav_gazebo_share, 'worlds', world)

    # Дополняем (не затираем) GZ_SIM_RESOURCE_PATH: нужны и наши модели
    # (h_marker), и модели PX4 (x500, ground_plane и т.д.).
    models_path = os.path.join(uav_gazebo_share, 'models')
    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    resource_path = os.pathsep.join([models_path, px4_models])
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        resource_path += os.pathsep + existing

    env = dict(os.environ)
    # px4-rc.simulator запускает gz как "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf",
    # где PX4_GZ_WORLDS указывает на Tools/simulation/gz/worlds (gz_env.sh).
    # Поэтому кладём наш мир туда под именем <world>.sdf и передаём только имя.
    world_name = os.path.splitext(os.path.basename(world))[0]
    px4_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
    try:
        os.makedirs(px4_worlds, exist_ok=True)
        shutil.copyfile(world_path, os.path.join(px4_worlds, world_name + '.sdf'))
    except OSError as exc:
        print(f'[uav_bringup] не удалось скопировать мир в {px4_worlds}: {exc}')

    env['PX4_GZ_WORLD'] = world_name
    env['GZ_SIM_RESOURCE_PATH'] = resource_path
    if context.launch_configurations.get('headless', 'false').lower() in ('true', '1'):
        env['HEADLESS'] = '1'

    return [
        ExecuteProcess(
            cmd=['make', 'px4_sitl', px4_model],
            cwd=px4_dir,
            env=env,
            output='screen',
        )
    ]


def _launch_offboard(context, offboard_params, nav2_commander_params, use_sim_time):
    """Выбор offboard-ноды по ``mission_mode``.

    * ``points`` — миссия по точкам (offboard_node, trajectory_setpoint/MPC);
    * ``nav2``   — следование за Nav2 (/cmd_vel -> position setpoint) с
                  force-disarm после подтверждения касания земли.
    """
    mode = context.launch_configurations.get('mission_mode', 'points')
    takeoff_height = context.launch_configurations.get('takeoff_height', '3.0')
    if mode == 'nav2':
        return [Node(
            package='uav_offboard',
            executable='nav2_commander_node',
            name='nav2_commander_node',
            parameters=[nav2_commander_params,
                        {'use_sim_time': use_sim_time},
                        {'takeoff_height': float(takeoff_height)}],
            output='screen',
        )]
    return [Node(
        package='uav_offboard',
        executable='offboard_node',
        name='offboard_node',
        parameters=[offboard_params],
        output='screen',
    )]


def generate_launch_description() -> LaunchDescription:
    # --- Share-каталоги пакетов ---
    uav_gazebo_share = get_package_share_directory('uav_gazebo')
    uav_sensors_share = get_package_share_directory('uav_sensors')
    uav_navigation_share = get_package_share_directory('uav_navigation')
    uav_offboard_share = get_package_share_directory('uav_offboard')
    uav_perception_share = get_package_share_directory('uav_perception')

    # --- Пути к конфигам ---
    bridge_config = PathJoinSubstitution(
        [uav_sensors_share, 'config', 'gazebo_bridge.yaml'])
    nav2_params = PathJoinSubstitution(
        [uav_navigation_share, 'config', 'nav2_params.yaml'])
    offboard_params = PathJoinSubstitution(
        [uav_offboard_share, 'config', 'offboard.yaml'])
    nav2_commander_params = PathJoinSubstitution(
        [uav_offboard_share, 'config', 'nav2_commander.yaml'])
    perception_params = PathJoinSubstitution(
        [uav_perception_share, 'config', 'perception.yaml'])

    # --- Трек траектории дрона (gz-transport -> /marker) ---
    # project_root — корень проекта. Файл запускается из install/<pkg>/share/...
    # (colcon по умолчанию КОПИРУЕТ launch-файлы, поэтому realpath(__file__)
    # НЕ выводит в src, как предполагал старый код с 4 dirname). Ищем корень,
    # поднимаясь вверх по каталогам до каталога с tools/trajectory_trail.
    _root_candidate = os.path.dirname(os.path.realpath(__file__))
    project_root = os.getcwd()
    for _step in range(10):
        if os.path.isdir(
                os.path.join(_root_candidate, 'tools', 'trajectory_trail')):
            project_root = _root_candidate
            break
        _parent = os.path.dirname(_root_candidate)
        if _parent == _root_candidate:
            break
        _root_candidate = _parent
    trail_bin = os.path.join(project_root, 'tools', 'trajectory_trail',
                             'trajectory_trail')

    # --- Аргументы запуска (подстановки) ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_nav2 = LaunchConfiguration('start_nav2')
    mission_mode = LaunchConfiguration('mission_mode')
    start_trail = LaunchConfiguration('start_trail')

    # --- MicroXRCEAgent: мост uORB <-> ROS2 (БЕЗ MAVROS) ---
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    # --- Трек траектории: постоянный след за дроном в Gazebo GUI ---
    trail = ExecuteProcess(
        cmd=[trail_bin, '--pose-topic', '/model/x500_depth_0/pose',
             '--model', 'x500_depth_0'],
        condition=IfCondition(start_trail),
        output='screen',
    )

    # --- ros_gz_bridge: сенсоры камеры + /clock (для use_sim_time) ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # --- Offboard-управление (mission_mode: points | nav2) ---
    offboard_node = OpaqueFunction(
        function=_launch_offboard,
        kwargs={
            'offboard_params': offboard_params,
            'nav2_commander_params': nav2_commander_params,
            'use_sim_time': use_sim_time,
        },
    )

    # --- Глубина -> /scan (для Nav2) ---
    depth_to_scan = Node(
        package='uav_sensors',
        executable='depth_to_scan',
        name='depth_to_scan',
        output='screen',
    )

    # --- TF odom -> base_link (требуется Nav2, из VehicleOdometry NED->ENU).
    # ВАЖНО: use_sim_time, иначе TF штампуется wall-clock, а сенсоры — sim-временем,
    # из-за чего costmap отбрасывает сообщения ("earlier than all data in cache").
    odom_tf_broadcaster = Node(
        package='uav_sensors',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # --- Статическая TF base_link -> camera_depth_frame (для /scan).
    # Камера OakD-Lite в x500_depth закреплена на (.24 .06 .484), сенсор внутри
    # ещё на (.01233 -.03 .01878) -> итоговая позиция ~(0.252, 0.03, 0.503),
    # смотрит вперёд (+X) без поворота.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf',
        arguments=[
            '0.252', '0.03', '0.503', '0', '0', '0',
            'base_link', 'camera_depth_frame',
        ],
        output='screen',
    )

    # --- TF для «сырых» кадров сенсоров Gazebo (frame_id ros_gz_bridge).
    # Облако точек /camera/points приходит с frame_id
    # "x500_depth_0/OakD-Lite/base_link/StereoOV7251", а RGB /camera/image_raw —
    # "x500_depth_0/OakD-Lite/base_link/IMX214". Оба сенсора находятся в одной
    # точке (0.252, 0.03, 0.503) относительно base_link. Без этих TF VoxelLayer
    # Nav2 отбрасывает облако точек.
    camera_points_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_points_tf',
        arguments=[
            '0.252', '0.03', '0.503', '0', '0', '0',
            'base_link', 'x500_depth_0/OakD-Lite/base_link/StereoOV7251',
        ],
        output='screen',
    )
    camera_rgb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_rgb_tf',
        arguments=[
            '0.252', '0.03', '0.503', '0', '0', '0',
            'base_link', 'x500_depth_0/OakD-Lite/base_link/IMX214',
        ],
        output='screen',
    )

    # --- Компьютерное зрение ---
    marker_detector = Node(
        package='uav_perception',
        executable='marker_detector',
        name='marker_detector',
        parameters=[perception_params],
        output='screen',
    )
    human_detector = Node(
        package='uav_perception',
        executable='human_detector',
        name='human_detector',
        parameters=[perception_params],
        output='screen',
    )

    # --- Локализация цели: /detections + /camera/depth -> /target/position (odom).
    # ВАЖНО use_sim_time: TF и штампы сообщений живут в sim-времени, иначе
    # transform() с sim-штампом не найдёт трансформацию.
    target_localizer = Node(
        package='uav_perception',
        executable='target_localizer',
        name='target_localizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # --- Nav2: плоский навигационный стек (высоту держит offboard) ---
    nav2_group = GroupAction(
        condition=IfCondition(start_nav2),
        actions=[
            Node(package='nav2_controller',
                 executable='controller_server',
                 name='controller_server',
                 parameters=[nav2_params],
                 output='screen'),
            Node(package='nav2_planner',
                 executable='planner_server',
                 name='planner_server',
                 parameters=[nav2_params],
                 output='screen'),
            Node(package='nav2_behaviors',
                 executable='behavior_server',
                 name='behavior_server',
                 parameters=[nav2_params],
                 output='screen'),
            Node(package='nav2_bt_navigator',
                 executable='bt_navigator',
                 name='bt_navigator',
                 parameters=[nav2_params],
                 output='screen'),
            Node(package='nav2_waypoint_follower',
                 executable='waypoint_follower',
                 name='waypoint_follower',
                 parameters=[nav2_params],
                 output='screen'),
            Node(package='nav2_lifecycle_manager',
                 executable='lifecycle_manager',
                 name='lifecycle_manager_navigation',
                 parameters=[{'use_sim_time': use_sim_time},
                             {'autostart': autostart},
                             {'node_names': NAV2_LIFECYCLE_NODES}],
                 output='screen'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='lab.world',
            description='Имя мира Gazebo (в share/uav_gazebo/worlds/)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Использовать симуляционное время Gazebo'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Автоматически активировать lifecycle Nav2'),
        DeclareLaunchArgument(
            'start_px4', default_value='true',
            description='Запускать PX4 SITL (выключите, если уже запущен)'),
        DeclareLaunchArgument(
            'start_nav2', default_value='true',
            description='Запускать стек Nav2'),
        DeclareLaunchArgument(
            'mission_mode', default_value='points',
            description='Режим миссии: points (по точкам) или nav2 (следование '
                        'за Nav2 через /navigate_to_pose)'),
        DeclareLaunchArgument(
            'takeoff_height', default_value='3.0',
            description='Высота взлёта/удержания в режиме nav2 [м] '
                        '(зазор над машинами ~1.5 м при 3.0)'),
        DeclareLaunchArgument(
            'start_trail', default_value='true',
            description='Рисовать постоянный трек траектории дрона '
                        '(trajectory_trail -> /marker)'),
        DeclareLaunchArgument(
            'px4_model', default_value='gz_x500_depth',
            description='Модель PX4 SITL для Gazebo '
                        '(gz_x500_depth — с камерой OakD-Lite)'),
        DeclareLaunchArgument(
            'px4_dir', default_value='',
            description='Путь к PX4-Autopilot '
                        '(по умолчанию $PX4_AUTOPILOT_DIR или ./PX4-Autopilot)'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Запуск Gazebo без GUI (HEADLESS=1)'),
        LogInfo(msg=['Запуск полной симуляции UAV']),
        microxrce_agent,
        trail,
        OpaqueFunction(
            function=_launch_px4,
            kwargs={'uav_gazebo_share': uav_gazebo_share}),
        gz_bridge,
        offboard_node,
        depth_to_scan,
        odom_tf_broadcaster,
        camera_tf,
        camera_points_tf,
        camera_rgb_tf,
        marker_detector,
        human_detector,
        target_localizer,
        nav2_group,
    ])
