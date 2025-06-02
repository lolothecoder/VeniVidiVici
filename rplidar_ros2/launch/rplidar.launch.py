from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[('/scan', '/scan_raw')]
        ),
        Node(
            package='laser_filters',
            executable='scan_filter_chain',
            name='scan_filter_chain',
            parameters=[{'scan_filter_chain': 'VeniVidiVici/rplidar_ros2/config/lidar_filter.yaml'}],
            remappings=[
                ('scan_in', 'scan_raw'),     # subscribe to raw 360° data
                ('scan_out', 'scan')         # publish the cropped scan onto /scan
            ]
        ),
    ])
