from setuptools import setup
import os
from glob import glob

package_name = 'dexi_led'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pi5neo'],
    zip_safe=True,
    maintainer='Dennis Baldwin',
    maintainer_email='db@droneblocks.io',
    description='LED control package for DEXI drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_service_pi5 = dexi_led.led_service_pi5:main',
            'led_service_cm4 = dexi_led.led_service_cm4:main',
            'led_flight_mode_status = dexi_led.led_flight_mode_status:main',
            'led_visualization_bridge = dexi_led.led_visualization_bridge:main',
        ],
    },
)