import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'track_visualizer'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Define paths
    mesh_path = os.path.join(pkg_share, 'meshes', 'track.dae')

    return LaunchDescription([
        # 1. Static TF: map -> track_frame (Zero transform)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='track_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'track_frame']
        ),

        # 2. Path Publisher
        Node(
            package=pkg_name,
            executable='path_publisher',
            name='path_publisher',
            output='screen'
        ),

        # 3. PID Controller
        Node(
            package=pkg_name,
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        ),

        # 4. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])