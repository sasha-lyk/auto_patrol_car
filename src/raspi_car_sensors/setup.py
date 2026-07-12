import os
from glob import glob
from setuptools import setup

package_name = 'raspi_car_sensors'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykctj',
    maintainer_email='ykctj@example.com',
    description='Sensor drivers (YB_MRA02 IMU) for the Raspberry Pi ROS2 car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yb_mra02_uart_node = raspi_car_sensors.yb_mra02_uart_node:main',
        ],
    },
)
