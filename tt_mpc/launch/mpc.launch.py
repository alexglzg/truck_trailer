from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The MPC Controller
        Node(
            package='tt_mpc',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            # parameters=[
            #     {'control_rate': 20.0}
            # ]
        ),
        
        # 2. The Obstacle Publisher (and Visualizer)
        # Node(
        #     package='tt_mpc',
        #     executable='obstacle_publisher',
        #     name='obstacle_publisher',
        #     output='screen'
        # )
    ])