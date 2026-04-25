import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'button_mapping_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        ('lib/' + package_name, [
            'database_interface.py',
            'database.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adorjan',
    maintainer_email='your_email@example.com',
    description='ROS2 package for managing button and joystick mappings',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_listener = button_mapping_ros.mapping_listener:main',
        ],
    },
)
