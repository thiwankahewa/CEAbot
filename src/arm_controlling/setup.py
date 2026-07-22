from setuptools import find_packages, setup

package_name = 'arm_controlling'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thiwa',
    maintainer_email='thiwankahewavitharana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'plant_locator = arm_controlling.plant_locator:main',
        'plant_view_scanner = arm_controlling.plant_view_scanner:main',
        'arm_manager = arm_controlling.arm_manager:main',
        'joint_rotation_monitor = arm_controlling.joint_rotation_monitor:main',
    ],
},
)
