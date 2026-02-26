from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_node = Node(
        package='fake_data',
        executable='fake_imu',
        output='screen',
    )

    gps_node = Node(
        package='fake_data',
        executable='fake_gps',
        output='screen',
    )

    return LaunchDescription([
        imu_node,
        gps_node,
    ])
