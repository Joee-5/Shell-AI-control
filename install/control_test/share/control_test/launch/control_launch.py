from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Path Publisher Node (The Track)
        Node(
            package='control_test',
            executable='path_node',
            name='path_node'
        ),
        
        # 2. PID Longitudinal Node (Throttle Control)
        Node(
            package='control_test',
            executable='pid_node',
            name='pid_node'
        ),
        
        # 3. Pure Pursuit Lateral Node (Steering Control)
        Node(
            package='control_test',
            executable='pure_node',
            name='pure_node'
        ),

        # 4. Odometry & Physics Node (The "Virtual" Car)
        # This node provides the TF transform to solve the Fixed Frame issue
        Node(
            package='control_test',
            executable='odom_node',
            name='odom_node'
        ),

        # 5. Open RViz2 automatically
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        )
    ])