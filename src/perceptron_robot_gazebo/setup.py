from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perceptron_robot_gazebo'


def model_data_files():
    """Install models/ recursively, preserving the directory layout.

    Gazebo resolves model://<name>/... against GAZEBO_MODEL_PATH, so the tree
    under share/<pkg>/models has to mirror the source tree exactly, including
    materials/scripts and materials/textures.
    """
    entries = []
    for root, _dirs, files in os.walk('models'):
        if not files:
            continue
        entries.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Every file under models/, at whatever depth. The previous version
        # listed aruco_dock_station's three directories by hand, so a new model
        # was silently not installed: model:// failed to resolve, Gazebo carried
        # on without it, and the robot fell through a world with no ground.
        *model_data_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yadi',
    maintainer_email='yadi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
