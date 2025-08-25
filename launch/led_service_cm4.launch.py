from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the CM4 LED service node
    led_service_node = Node(
        package='dexi_led',
        executable='led_service_cm4',
        name='led_service',
        namespace='dexi',
        output='screen'
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
        led_service_node,
        flight_mode_status_node
    ])