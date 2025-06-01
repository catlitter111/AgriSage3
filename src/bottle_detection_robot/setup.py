from setuptools import setup
import os
from glob import glob

package_name = 'bottle_detection_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xls')),
        (os.path.join('share', package_name, 'models'), glob('models/*.rknn')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2功能包用于瓶子检测和机器人控制',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = bottle_detection_robot.camera_node:main',
            'bottle_detector_node = bottle_detection_robot.bottle_detector_node:main',
        ],
    },
)