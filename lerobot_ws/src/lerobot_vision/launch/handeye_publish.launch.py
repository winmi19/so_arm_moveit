"""发布手眼标定结果 TF

标定完成后使用, 从 ~/.ros/easy_handeye2/ 读取标定结果并发布为静态 TF。

Usage:
    ros2 launch lerobot_vision handeye_publish.launch.py name:=so101_eih
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name_arg = DeclareLaunchArgument(
        'name', default_value='so101_eih',
        description='标定名称 (需与 calibrate 时一致)'
    )

    handeye_publisher = Node(
        package='easy_handeye2',
        executable='handeye_publisher',
        name='handeye_publisher',
        parameters=[{
            'name': LaunchConfiguration('name'),
        }],
    )

    return LaunchDescription([
        name_arg,
        handeye_publisher,
    ])
