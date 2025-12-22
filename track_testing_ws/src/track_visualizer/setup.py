import os
from glob import glob
from setuptools import setup

package_name = 'track_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 🚨 LAUNCH FILE INSTALLATION (This must be correct) 🚨
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'track_launch.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omar',
    maintainer_email='omar@example.com',
    description='ROS 2 package for track visualization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 🚨 PYTHON SCRIPT DEFINITIONS 🚨
            'path_publisher = track_visualizer.path_publisher:main',
            'track_model_publisher = track_visualizer.track_model_publisher:main',
        ],
    },
)