#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'led_count',
            default_value='78',
            description='Number of LEDs in the ring'
        ),
        DeclareLaunchArgument(
            'brightness', 
            default_value='0.5',
            description='LED brightness (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'visualization_radius',
            default_value='1.0',
            description='Radius of LED ring in meters'
        ),
        DeclareLaunchArgument(
            'visualization_height',
            default_value='2.0', 
            description='Height of LED ring above ground in meters'
        ),
        DeclareLaunchArgument(
            'led_marker_size',
            default_value='0.04',
            description='Size of LED markers in meters'
        ),
        
        # LED visualization bridge node
        Node(
            package='dexi_led',
            executable='led_visualization_bridge',
            name='led_service',
            namespace='dexi',
            parameters=[{
                'led_count': LaunchConfiguration('led_count'),
                'brightness': LaunchConfiguration('brightness'),
                'visualization_radius': LaunchConfiguration('visualization_radius'),
                'visualization_height': LaunchConfiguration('visualization_height'),
                'led_marker_size': LaunchConfiguration('led_marker_size'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])