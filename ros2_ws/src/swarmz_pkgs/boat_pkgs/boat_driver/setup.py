from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'boat_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesus',
    maintainer_email='jesus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boat_client_demo = '+package_name+'.boat_client_demo:main',
            'cannon_server = '+package_name+'.cannon_server:main',
            'gz_tracker = '+package_name+'.gz_tracker:main'
        ],
    },
)
