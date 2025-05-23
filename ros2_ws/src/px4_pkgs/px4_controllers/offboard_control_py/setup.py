import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'offboard_control_py'

# Find all Python files in the demo_clients directory and subdirectories to register as executables
demo_client_scripts = []

# Walk through the demo_clients directory and its subdirectories
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
            demo_client_scripts.append(entry_point)

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
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'px4_msgs',
        'px4_controllers_interfaces'
    ],
    zip_safe=True,
    maintainer='maintainer_name',
    maintainer_email='maintainer_email@example.com',
    description='Offboard control for PX4 using ROS 2',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = '+package_name+'.offboard_control:main',
            'offboard_control_px4 = '+package_name+'.offboard_control_px4:main',
            'offboard_control_client = '+package_name+'.offboard_control_client:main',
            'offboard_control_client_template = '+package_name+'.offboard_control_client_template:main',
            'speed_tester = '+package_name+'.speed_tester:main',
        ] + demo_client_scripts,
    },
)
