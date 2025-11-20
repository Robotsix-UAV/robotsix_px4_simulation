from setuptools import setup, find_packages
from glob import glob

package_name = 'robotsix_px4_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        # Package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        
        # Config files
        ('share/' + package_name + '/config/dds_topics',
            glob('config/dds_topics/*.yaml')),
        ('share/' + package_name + '/config/px4_parameters',
            glob('config/px4_parameters/*')),
        
        # Worlds
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        
        # Models
        ('share/' + package_name + '/models/common',
            glob('models/common/*')),
        ('share/' + package_name + '/models/propellers',
            glob('models/propellers/*.dae')),
        ('share/' + package_name + '/models/quad_gps',
            glob('models/quad_gps/*')),
        ('share/' + package_name + '/models/quad_mocap',
            glob('models/quad_mocap/*')),
        
        # Docker files
        ('share/' + package_name + '/docker',
            glob('docker/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damien Six',
    maintainer_email='six.damien@robotsix.net',
    description='ROS2 package for PX4 SITL simulation with Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_server = robotsix_px4_simulation.simulation_server:main',
        ],
    },
)