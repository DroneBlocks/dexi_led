from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    led_pin_arg = DeclareLaunchArgument(
        'led_pin',
        default_value='12',
        description='GPIO for the LED data line: 12 (ARK LED Strip port) or 21 (ARK GPIO port)'
    )

    # Create the CM4 LED service node
    led_service_node = Node(
        package='dexi_led',
        executable='led_service_cm4',
        name='led_service',
        namespace='dexi',
        output='screen',
        parameters=[{'led_pin': LaunchConfiguration('led_pin')}]
    )

    # Create the flight mode status node
    flight_mode_status_node = Node(
        package='dexi_led',
        executable='led_flight_mode_status',
        name='led_flight_mode_status',
        namespace='dexi',
        output='screen'
    )

    return LaunchDescription([
        led_pin_arg,
        led_service_node,
        flight_mode_status_node
    ])