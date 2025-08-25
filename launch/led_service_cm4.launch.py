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

    return LaunchDescription([
        led_service_node
    ])