import os
import tempfile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg      = FindPackageShare('warehouse')
    pkg_path = get_package_share_directory('warehouse')

    nav2_params_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    map_yaml_path    = os.path.join(pkg_path, 'maps',   'warehouse_map.yaml')

    # ── Clean SDF ─────────────────────────────────────────────────────────
    raw_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_burger', 'model.sdf'
    )
    with open(raw_sdf, 'r') as f:
        sdf_content = f.read()
    sdf_clean = sdf_content.replace('${namespace}', '')
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False, prefix='tb3_')
    tmp.write(sdf_clean)
    tmp.flush()
    clean_sdf_path = tmp.name

    # ── Clean URDF ────────────────────────────────────────────────────────
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('${namespace}', '')

    # ── 1  Gazebo ──────────────────────────────────────────────────────────
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'spawn_warehouse.launch.py'])
        )
    )

    # ── 2  Clock bridge ────────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # ── 3  Robot state publisher ───────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ── 4  Spawn robot at 8 s ──────────────────────────────────────────────
    spawn = TimerAction(
        period=8.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name',  'tb3',
                '-file',  clean_sdf_path,
                '-world', 'warehouse',
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
            ],
        )]
    )

    # ── 5  Topic bridges at 12 s ───────────────────────────────────────────
    robot_bridge = TimerAction(
        period=12.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='robot_bridge',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            parameters=[{'use_sim_time': True}]
        )]
    )

    # ── 6  odom→base_footprint TF at 13 s ─────────────────────────────────
    odom_tf = TimerAction(
        period=13.0,
        actions=[Node(
            package='warehouse',
            executable='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    # ── 7  Map server at 14 s ──────────────────────────────────────────────
    #       Loads warehouse_map.pgm and publishes /map
    map_server = TimerAction(
        period=14.0,
        actions=[Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time':  True,
                'yaml_filename': map_yaml_path,
            }]
        )]
    )

    # ── 8  AMCL at 15 s ───────────────────────────────────────────────────
    #       Localizes robot in the loaded map → publishes map→odom
    amcl = TimerAction(
        period=15.0,
        actions=[Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': True}]
        )]
    )

    # ── 9  Lifecycle manager for map_server + amcl at 16 s ────────────────
    lc_map = TimerAction(
        period=16.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'node_names':   ['map_server', 'amcl'],
            }]
        )]
    )

    # ── 10  Nav2 full stack at 18 s ────────────────────────────────────────
    nav2 = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}],
                remappings=[('cmd_vel', '/cmd_vel_nav')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': True}],
                remappings=[
                    ('cmd_vel',          '/cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/cmd_vel'),
                ]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart':    True,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ],
                }]
            ),
        ]
    )

    # ── 11  RViz2 at 20 s ─────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz'
    )
    rviz = TimerAction(
        period=20.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        )]
    )

    # ── 12  Auto goal sender at 25 s ──────────────────────────────────────
    #        Sends the robot autonomously to the finish line (x=17, y=0)
    goal_sender = TimerAction(
        period=25.0,
        actions=[Node(
            package='warehouse',
            executable='send_goal',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_x':  17.0,
                'goal_y':   0.0,
                'goal_yaw': 0.0,
            }]
        )]
    )

    return LaunchDescription([
        world,         #  0 s — Gazebo
        clock_bridge,  #  0 s — /clock
        rsp,           #  0 s — robot_state_publisher
        spawn,         #  8 s — spawn turtlebot
        robot_bridge,  # 12 s — topic bridges
        odom_tf,       # 13 s — odom→base_footprint TF
        map_server,    # 14 s — load warehouse_map.pgm
        amcl,          # 15 s — localize robot in map
        lc_map,        # 16 s — activate map_server + amcl
        nav2,          # 18 s — full Nav2 stack
        rviz,          # 20 s — visualisation
        goal_sender,   # 25 s — robot drives itself to finish line
    ])
