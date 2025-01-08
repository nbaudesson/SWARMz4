import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'offboard_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer_name',
    maintainer_email='maintainer_email@example.com',
    description='Offboard control for PX4 using ROS 2',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = '+package_name+'.offboard_control:main',
            'offboard_control_goto = '+package_name+'.offboard_control_goto:main',
            'simu_processes = '+package_name+'.simu_processes:main',
            'simu_circle = '+package_name+'.simu_circle:main',
            'mission_control = '+package_name+'.mission_control:main',
            'tight_circle = '+package_name+'.tight_circle:main',
        ],
    },
)
