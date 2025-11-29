from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Sensor Publisher'ı başlat
        Node(
            package='sensor_publisher_pkg',
            executable='sensor_publisher',
            output='screen'
        ),
        # 2. Data Processor'ı başlat
        Node(
            package='data_processor_pkg',
            executable='data_processor',
            output='screen'
        ),
        # 3. Command Server'ı başlat
        Node(
            package='command_server_pkg',
            executable='command_server',
            output='screen'
        ),
    ])
