import os
from glob import glob
from setuptools import setup

package_name = 'raspi_car_base'

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
    description='Raspberry Pi L298N differential base with quadrature encoders '
                'and closed-loop PID velocity control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_node = raspi_car_base.encoder_node:main',
            'wheel_odometry_node = raspi_car_base.wheel_odometry_node:main',
            'l298n_motor_node = raspi_car_base.l298n_motor_node:main',
        ],
    },
)
