from setuptools import setup

package_name = 'fake_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Fake robot package for GUI development',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_controller = fake_robot_pkg.fake_controller_node:main',
        ],
    },
)
