from setuptools import setup, find_packages
from setuptools.command.install import install
from glob import glob
import subprocess
import os

package_name = 'robotsix_px4_simulation'

class ProcessXacroInstall(install):
    """Custom install command to process xacro files."""
    
    def run(self):
        # Run the standard install
        install.run(self)
        
        # Process xacro files after installation
        self.process_xacro_files()
    
    def process_xacro_files(self):
        """Process all .xacro files to their output format."""
        install_base = self.install_data
        share_dir = os.path.join(install_base, 'share', package_name)
        models_dir = os.path.join(share_dir, 'models')
        
        # Find all .xacro files in the models directory
        xacro_files = []
        for root, dirs, files in os.walk(models_dir):
            for file in files:
                if file.endswith('.xacro'):
                    xacro_files.append(os.path.join(root, file))
        
        # Process each xacro file
        for xacro_file in xacro_files:
            # Determine output file (remove .xacro extension)
            output_file = xacro_file.rsplit('.xacro', 1)[0]
            
            try:
                print(f"Processing xacro: {xacro_file} -> {output_file}")
                subprocess.run(
                    ['xacro', xacro_file, '-o', output_file],
                    check=True,
                    capture_output=True,
                    text=True
                )
                print(f"Successfully processed: {output_file}")
            except subprocess.CalledProcessError as e:
                print(f"Warning: Failed to process {xacro_file}: {e}")
                if e.stderr:
                    print(f"Error output: {e.stderr}")
            except FileNotFoundError:
                print("Warning: xacro command not found. Skipping xacro processing.")
                print("Install xacro with: sudo apt-get install ros-*-xacro")
                break

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    cmdclass={
        'install': ProcessXacroInstall,
    },
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