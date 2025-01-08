import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'game_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nb_adm',
    maintainer_email='nb_adm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_master_node = game_master.game_master_node:main',
            'game_master_missile_server = game_master.game_master_missile_server:main',
            'game_master_kamikaze_server = game_master.game_master_kamikaze_server:main',
        ],
    },
)
