from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for Pi5
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
    
    led_driver_arg = DeclareLaunchArgument(
        'led_driver',
        default_value='spi',
        description="LED backend: 'spi' (SPI1 MOSI, GPIO 20) or 'pio' (RP1 PIO, any pin)"
    )

    led_pin_arg = DeclareLaunchArgument(
        'led_pin',
        default_value='12',
        description='GPIO pin for the pio driver (ignored by spi)'
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

    # Create the Pi5 LED service node
    led_service_node = Node(
        package='dexi_led',
        executable='led_service_pi5',
        name='led_service',
        namespace='dexi',
        parameters=[{
            'led_count': LaunchConfiguration('led_count'),
            'brightness': LaunchConfiguration('brightness'),
            'spi_speed': LaunchConfiguration('spi_speed'),
            'led_driver': LaunchConfiguration('led_driver'),
            'led_pin': LaunchConfiguration('led_pin'),
            'simulation_mode': LaunchConfiguration('simulation_mode')
        }],
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
        led_count_arg,
        brightness_arg,
        led_driver_arg,
        led_pin_arg,
        spi_speed_arg,
        simulation_mode_arg,
        led_service_node,
        flight_mode_status_node
    ])