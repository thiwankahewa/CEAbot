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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
         (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'arduino_bridge = bench_robot.arduino_bridge:main',
            'bench_tracker  = bench_robot.bench_tracker:main',
            'joystick_teleop = bench_robot.joystick_teleop:main',
            'motor_controller = bench_robot.motor_controller:main',
            'settings_saver = bench_robot.settings_saver:main',
            'hub_motor_driver = bench_robot.hub_motor_driver:main',
            'motor_control_mux = bench_robot.motor_control_mux:main',
        ],
    },
)
