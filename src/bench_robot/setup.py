from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bench_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiwa',
    maintainer_email='thiwa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = bench_robot.arduino_bridge_node:main',
            'bench_tracker_node  = bench_robot.bench_tracker_node:main',
            'joystick_teleop_node = bench_robot.joystick_teleop_node:main',
        ],
    },
)
