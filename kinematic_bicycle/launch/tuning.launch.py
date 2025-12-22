from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    base_path = get_package_share_directory('kinematic_bicycle')
    rviz_path = os.path.join(base_path, 'bicycle.rviz')
    
    return LaunchDescription([
        # Bicycle model
        Node(
            package='kinematic_bicycle',
            executable='kinematic_bicycle',
            name='bicycle_model'
        ),
        # Path generator
        Node(
            package='kinematic_bicycle', 
            executable='path_gen',
            name='path_generator'
        ),
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path]
        ),
    ])