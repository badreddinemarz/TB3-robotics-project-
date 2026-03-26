from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    world_path = PathJoinSubstitution([
        FindPackageShare('warehouse'),
        'worlds',
        'my_world.sdf'
    ])

    spawn_tb3 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'tb3_1',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_tb3
    ])

