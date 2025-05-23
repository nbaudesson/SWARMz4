from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'boat_driver'

# Find all Python files in the client_codes directory and subdirectories to register as executables
client_scripts = []

# Walk through the client_codes directory and its subdirectories
for root, dirs, files in os.walk(os.path.join(package_name, 'client_codes')):
    # Get the relative path from the package
    rel_path = os.path.relpath(root, package_name)
    # Convert path separator to dots for the import path
    import_path = rel_path.replace(os.sep, '.')
    
    # Process each Python file
    for file in files:
        if file.endswith('.py') and file != '__init__.py':
            # Get the module name without .py extension
            module_name = file[:-3]
            
            # The command name is just the filename without extension
            command_name = module_name
            
            # The full import path
            full_import_path = f"{package_name}.{import_path}.{module_name}"
            
            # Add the entry point
            entry_point = f"{command_name} = {full_import_path}:main"
            client_scripts.append(entry_point)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Add config files if needed
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'boat_client_template = '+package_name+'.boat_client_template:main',
            'boat_client_demo = '+package_name+'.boat_client_demo:main',
            'cannon_server = '+package_name+'.cannon_server:main',
            'gz_tracker = '+package_name+'.gz_tracker:main',
        ] + client_scripts,
    },
)