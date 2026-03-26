import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg = FindPackageShare('warehouse')
    pkg_path = get_package_share_directory('warehouse')
    model_sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_burger', 'model.sdf'
    )
    # ── 1  Launch Gazebo with your warehouse world ─────────────────────
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'spawn_warehouse.launch.py'])
        )
    )
    # ── 2  Clock bridge — start immediately ────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )
    # ── 3  Robot state publisher ───────────────────────────────────────
    tb3_desc_pkg = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc_pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )
    # ── 4  Spawn robot — wait 8 s for Gazebo to fully start ────────────
    #       ONE TimerAction only — the previous code had TimerAction(TimerAction())
    #       which caused the spawn Node to never execute.
    spawn = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'tb3',
                    '-file', model_sdf_path,
                    '-world', 'warehouse',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01',
                ],
            )
        ]
    )
    # ── 5  ROS-Gazebo topic bridge — wait 12 s (after robot is spawned) ─
    robot_bridge = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='robot_bridge',
                output='screen',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                ],
                parameters=[{
                    'use_sim_time': True,
                }]
            )
        ]
    )
    return LaunchDescription([
        world,
        clock_bridge,
        rsp,
        spawn,         # single TimerAction at 8 s
        robot_bridge,  # single TimerAction at 12 s (after spawn)
    ])
