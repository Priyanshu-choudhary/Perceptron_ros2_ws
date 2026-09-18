import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ct6b_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laptop',
    maintainer_email='laptop@todo.todo',
    description='FlySky FS-CT6B RC transmitter teleop node for ROS 2 cmd_vel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ct6b_teleop_node = ct6b_teleop.ct6b_teleop_node:main',
        ],
    },
)
