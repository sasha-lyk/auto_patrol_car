import os
from glob import glob
from setuptools import setup

package_name = 'raspi_car_bringup'

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
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm') + glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykctj',
    maintainer_email='ykctj@example.com',
    description='Bringup: EKF fusion, SLAM, Nav2, patrol and mux for the encoder car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_mux_node = raspi_car_bringup.cmd_vel_mux_node:main',
            'turn_assist_node = raspi_car_bringup.turn_assist_node:main',
            'patrol_node = raspi_car_bringup.patrol_node:main',
            'wall_follower_node = raspi_car_bringup.wall_follower_node:main',
        ],
    },
)
