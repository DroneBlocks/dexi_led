from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    led_count_arg = DeclareLaunchArgument(
        'led_count',
        default_value='78',
        description='Number of LEDs in the strip'
    )
    
    brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='0.2',
        description='LED brightness (0.0-1.0)'
    )
    
    spi_speed_arg = DeclareLaunchArgument(
        'spi_speed',
        default_value='800',
        description='SPI communication speed'
    )
    
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode'
    )

    # Create the node
    led_service_node = Node(
        package='dexi_led',
        executable='led_service',
        name='led_service',
        namespace='dexi',
        parameters=[{
            'led_count': LaunchConfiguration('led_count'),
            'brightness': LaunchConfiguration('brightness'),
            'spi_speed': LaunchConfiguration('spi_speed'),
            'simulation_mode': LaunchConfiguration('simulation_mode')
        }],
        output='screen'
    )

    return LaunchDescription([
        led_count_arg,
        brightness_arg,
        spi_speed_arg,
        simulation_mode_arg,
        led_service_node
    ]) 