import os
from glob import glob
from setuptools import setup

package_name = 'raspi_car_web'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors'],
    zip_safe=True,
    maintainer='ykctj',
    maintainer_email='ykctj@example.com',
    description='Flask ROS2 web bridge + dashboard for the encoder car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'backend_ros2 = raspi_car_web.backend_ros2:main',
        ],
    },
)
