from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for CM4
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='dexi',
        description='Namespace for the LED service (e.g., dexi, dexi1, dexi2)'
    )

    # Create the CM4 LED service node
    led_service_node = Node(
        package='dexi_led',
        executable='led_service_cm4',
        name='led_service',
        namespace=LaunchConfiguration('namespace'),
        output='screen'
    )

    # Create the flight mode status node
    flight_mode_status_node = Node(
        package='dexi_led',
        executable='led_flight_mode_status',
        name='led_flight_mode_status',
        namespace=LaunchConfiguration('namespace'),
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        led_service_node,
        flight_mode_status_node
    ])