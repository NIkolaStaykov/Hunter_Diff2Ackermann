from setuptools import setup
import os
from glob import glob

package_name = 'ackermann_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    description='A ROS2 package to convert differential drive commands to Ackermann drive commands',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.service')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    entry_points={
        'console_scripts': [
            'control_transformer = ackermann_bridge.control_transformer:main',
        ],
    },
)

