from setuptools import setup
from glob import glob
import os

package_name = 'flag_ship'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Package.xml
        ('share/' + package_name, ['package.xml']),

        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

        # SDF files
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*.sdf')),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Mesh files (.stl)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas',
    maintainer_email='nicolas.baudesson@alten.com',
    description='flag_ship package with URDF, SDF, and meshes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
