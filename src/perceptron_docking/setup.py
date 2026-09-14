import os
from glob import glob
from setuptools import setup

package_name = 'perceptron_docking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Perceptron Developer',
    maintainer_email='dev@todo.todo',
    description='Precision ArUco-based self docking system for Perceptron robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = perceptron_docking.aruco_detector_node:main',
            'docking_controller_node = perceptron_docking.docking_controller_node:main',
            'dock = perceptron_docking.dock_client:main',
        ],
    },
)
