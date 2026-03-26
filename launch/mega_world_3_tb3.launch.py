from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def spawn_one(name: str, x: float, y: float, z: float):
    # Spawns a model into Gazebo from the URDF published on /<name>/robot_description
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', name,
            '-topic', f'/{name}/robot_description',
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
        ],
    )


def generate_launch_description():
    pkg = FindPackageShare('warehouse')

    # 1) Launch Gazebo with your world (uses your existing file)
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'spawn_warehouse.launch.py'])
        )
    )

    # 2) Start 3 robot_state_publishers in namespaces (uses your existing namespaced description launch)
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

    # 3) Delay spawn so Gazebo is up AND the robot_descriptions exist
    spawns = TimerAction(
        period=6.0,
        actions=[
            spawn_one('tb3_0', 0.0, 0.0, 0.01),
            spawn_one('tb3_1', 1.0, 0.0, 0.01),
            spawn_one('tb3_2', 0.0, 1.0, 0.01),
        ]
    )

    return LaunchDescription([
        world,
        desc0, desc1, desc2,
        spawns
    ])
