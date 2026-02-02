import os  # <--- THIS WAS MISSING
from glob import glob
from setuptools import setup

package_name = 'control_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs the launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='Control test project with PID and Pure Pursuit',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_node = control_test.pid:main',
            'pure_node = control_test.pure:main',
            'path_node = control_test.path:main',
            'odom_node = control_test.odom:main',
            'pid_tuner = control_test.pid_tuner:main',
        ],
    },
)
