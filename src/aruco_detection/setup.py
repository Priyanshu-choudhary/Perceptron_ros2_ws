from setuptools import setup

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Aruco marker publisher node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arUcoPosePublisher = aruco_detection.arUcoPosePublisher:main',
        ],
    },
)
