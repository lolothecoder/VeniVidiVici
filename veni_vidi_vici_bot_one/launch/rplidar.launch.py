from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

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
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            # remappings=[('scan', 'scan_raw')]
        ),
        
        # Node(
        #     package="laser_filters",
        #     executable="scan_to_scan_filter_chain",
        #     parameters=[
        #         PathJoinSubstitution([
        #             get_package_share_directory("veni_vidi_vici_bot_one"),
        #             "config", "lidar_filter.yaml",
        #         ])],
        #     # remappings=[
        #     #     ('scan', 'scan_raw'),        
        #     #     ('scan_filtered', 'scan')
        #     # ]
        # )
    ])
