import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tb3_pkg = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_pkg, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([rsp])


