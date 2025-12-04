#!/usr/bin/env python3
"""
MPPI planner launch file
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get package directory
    pkg_dir = get_package_share_directory('truck_trailer_sim')

    # truck-Trailer MPPI Planner
    planner = Node(
        package='truck_trailer_sim',
        executable='mppi_planner_node',
        name='truck_trailer_mppi_planner',
        output='screen'
    )

    return LaunchDescription([
        planner
    ])