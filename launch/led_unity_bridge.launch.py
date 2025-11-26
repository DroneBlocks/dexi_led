from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch the LED Unity Bridge for SITL visualization"""

    # Declare arguments
    led_count_arg = DeclareLaunchArgument(
        'led_count',
        default_value='45',
        description='Number of LEDs in the ring'
    )

    brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='0.2',
        description='Global brightness (0.0 to 1.0)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='15.0',
        description='LED state publish rate in Hz'
    )

    # LED Unity Bridge Node
    led_unity_bridge_node = Node(
        package='dexi_led',
        executable='led_unity_bridge',
        name='led_service',
        namespace='dexi',
        output='screen',
        parameters=[{
            'led_count': LaunchConfiguration('led_count'),
            'brightness': LaunchConfiguration('brightness'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    return LaunchDescription([
        led_count_arg,
        brightness_arg,
        publish_rate_arg,
        led_unity_bridge_node,
    ])
