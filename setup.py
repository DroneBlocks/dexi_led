from setuptools import setup

package_name = 'dexi_led'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'led_service = dexi_led.led_service:main',
        ],
    },
)