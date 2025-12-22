import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'track_visualizer'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # 1. Path Publisher Node (The main script creating the path)
    path_publisher_node = Node(
        package=pkg_name,
        executable='path_publisher.py',
        name='path_publisher',
        output='screen',
        emulate_tty=True
    )
    
    # 2. Static TF Publisher Node (Locates the track model relative to 'map')
    tf_publisher_node = Node(
        package=pkg_name,
        executable='track_model_publisher.py',
        name='track_model_publisher',
        output='screen'
    )

    # 3. RViz 2 Node (Starts the visualization tool)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_client',
        output='screen',
        # Optional: You can provide a custom config file here if you create one
        # arguments=['-d', os.path.join(pkg_share_dir, 'config', 'default.rviz')],
    )

    return LaunchDescription([
        path_publisher_node,
        tf_publisher_node,
        rviz_node
    ])