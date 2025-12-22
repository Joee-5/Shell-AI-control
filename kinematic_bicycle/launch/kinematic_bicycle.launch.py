from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory('kinematic_bicycle'))
    rviz_path = base_path + '/bicycle.rviz'
    
    return LaunchDescription([
        # 1. Kinematic Bicycle Model Node
        Node(
            package='kinematic_bicycle',
            namespace='car',
            executable='kinematic_bicycle',
            name='bicycle_model'
        ),
        # 2. Path Generator Node
        Node(
            package='kinematic_bicycle',
            namespace='',
            executable='path_gen',
            name='path_generator'
        ),
        # 3. CONTROLLER NODE - ADD THIS
        Node(
            package='kinematic_bicycle',
            namespace='',
            executable='test',
            name='controller_node'
        ),
        # 4. RViz
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            output='screen', 
            arguments=['-d', rviz_path]  # Fixed: added space between -d and path
        )
    ])