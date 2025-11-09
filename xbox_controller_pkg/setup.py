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
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.json'))),
        # Include udev rules
        (os.path.join('share', package_name, 'udev'),
            glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for Xbox 360 controller integration with button event publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_controller_node = xbox_controller_pkg.xbox_controller_node:main',
            'test_controller = xbox_controller_pkg.test_controller:main',
            'controller_diagnostics = xbox_controller_pkg.controller_diagnostics:main',
        ],
    },
)