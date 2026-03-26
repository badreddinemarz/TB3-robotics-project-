import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='tb3_0')
    declare_prefix = DeclareLaunchArgument('frame_prefix', default_value='tb3_0/')

    tb3_pkg = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_pkg, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # IMPORTANT: namespace makes the parameter live under /<namespace>/robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': frame_prefix,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        declare_namespace,
        declare_prefix,
        rsp
    ])

