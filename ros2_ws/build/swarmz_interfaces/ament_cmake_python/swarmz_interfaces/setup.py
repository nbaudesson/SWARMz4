from setuptools import find_packages
from setuptools import setup

setup(
    name='swarmz_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('swarmz_interfaces', 'swarmz_interfaces.*')),
)
