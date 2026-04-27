from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xbox_controller_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for Xbox 360 controller integration with button event publishing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = xbox_controller_pkg.controller_node:main',
        ],
    },
)