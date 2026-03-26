
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Your package with the world + urdf
    warehouse_pkg = FindPackageShare('warehouse')

    # 1) Start Gazebo Sim with the SDF world (same idea as spawn_warehouse.launch.py)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                warehouse_pkg,
                'worlds',
                'my_world.sdf'
            ])
        }.items()
    )

    # 2) Start robot_state_publisher to publish /robot_description (same as tb3_description.launch.py)
    tb3_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                warehouse_pkg,
                'launch',
                'tb3_description.launch.py'
            ])
        )
    )

    # 3) Spawn TB3 into Gazebo from /robot_description (same as spawn_tb3.launch.py)
    spawn_tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                warehouse_pkg,
                'launch',
                'spawn_tb3.launch.py'
            ])
        )
    )

    # Delay spawn a bit so Gazebo is up + /robot_description exists
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_tb3_launch]
    )

    return LaunchDescription([
        gazebo_launch,
        tb3_description_launch,
        delayed_spawn
    ])