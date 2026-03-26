import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_one_from_sdf(name: str, x: float, y: float, z: float, model_sdf):
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', name,
            '-file', model_sdf,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
        ],
    )


def bridge_robot(name: str):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{name}',
        output='screen',
        arguments=[
            f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/model/{name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/model/{name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        remappings=[
            (f'/model/{name}/cmd_vel', f'/{name}/cmd_vel'),
            (f'/model/{name}/odometry', f'/{name}/odom'),
            (f'/model/{name}/scan', f'/{name}/scan'),
        ]
    )


def generate_launch_description():
    pkg = FindPackageShare('warehouse')

    # Resolved as a real Python string so it passes correctly to Node arguments
    pkg_path = get_package_share_directory('warehouse')
    model_sdf_path = os.path.join(pkg_path, 'models', 'turtlebot3_burger', 'model.sdf')

    # 1 - Launch Gazebo with warehouse world
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'spawn_warehouse.launch.py'])
        )
    )

    # 2 - Robot state publishers (TF + robot_description per robot)
    desc0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'tb3_description_ns.launch.py'])
        ),
        launch_arguments={'namespace': 'tb3_0', 'frame_prefix': 'tb3_0/'}.items()
    )
    desc1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'tb3_description_ns.launch.py'])
        ),
        launch_arguments={'namespace': 'tb3_1', 'frame_prefix': 'tb3_1/'}.items()
    )
    desc2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'tb3_description_ns.launch.py'])
        ),
        launch_arguments={'namespace': 'tb3_2', 'frame_prefix': 'tb3_2/'}.items()
    )

    # 3 - Spawn robots after Gazebo boots
    spawns = TimerAction(
        period=4.0,
        actions=[
            spawn_one_from_sdf('tb3_0', 0.0, 0.0, 0.05, model_sdf_path),
            spawn_one_from_sdf('tb3_1', 2.0, 0.0, 0.05, model_sdf_path),
            spawn_one_from_sdf('tb3_2', -2.0, 0.0, 0.05, model_sdf_path),
        ]
    )

    # 4 - Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # 5 - Robot bridges (delayed to ensure robots are spawned first)
    bridge0 = TimerAction(
        period=8.0,
        actions=[bridge_robot('tb3_0')]
    )
    bridge1 = TimerAction(
        period=8.0,
        actions=[bridge_robot('tb3_1')]
    )
    bridge2 = TimerAction(
        period=8.0,
        actions=[bridge_robot('tb3_2')]
    )

    return LaunchDescription([
        world,
        desc0, desc1, desc2,
        spawns,
        clock_bridge,
        bridge0, bridge1, bridge2,
    ])