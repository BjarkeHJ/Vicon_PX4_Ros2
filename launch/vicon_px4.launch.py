import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vicon_px4')
    default_cfg = os.path.join(pkg_share, 'config', 'vicon_px4_config.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_cfg,
        description='Path to parameter YAML',
    )

    node = Node(
        package='vicon_px4',
        executable='vicon_px4_node',
        name='vicon_px4_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])