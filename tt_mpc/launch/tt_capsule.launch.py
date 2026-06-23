#!/usr/bin/env python3
"""
tt_capsule.launch.py  --  bring up the capsule-DCBF MPC stack.

Loads config/tt_capsule.yaml into all three new nodes. The `mode` argument
selects the source of /truck_trailer/state and the obstacle feed:
    mode:=sim       simulation_node + a sim obstacle source
    mode:=hardware  the real estimator + Vive-tracked obstacle (no sim)

The sim/hardware bring-up nodes live in your other packages
(truck_trailer_sim, tt_localization); fill in the exact executable names below.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tt_mpc'), 'config', 'tt_capsule.yaml')
    mode = LaunchConfiguration('mode')

    mpc = Node(package='tt_mpc', executable='mpc_capsule_node',
               name='mpc_capsule_node', output='screen', parameters=[cfg])
    manager = Node(package='tt_mpc', executable='capsule_obstacle_manager',
                   name='capsule_obstacle_manager', output='screen', parameters=[cfg])
    viz = Node(package='tt_mpc', executable='capsule_viz_node',
               name='capsule_viz_node', output='screen', parameters=[cfg])

    # --- sim bring-up (adjust package/executable to your tree) ---
    sim_plant = Node(package='truck_trailer_sim', executable='simulation_node',
                     name='simulation_node', output='screen',
                     parameters=[cfg],
                     condition=IfCondition(_is(mode, 'sim')))
    # a sim obstacle publishing nav_msgs/Odometry on /human1/odom_smoothed.
    # sim_obstacle = Node(package='truck_trailer_sim', executable='sim_obstacle_node',
    #                     name='sim_obstacle_node', output='screen', parameters=[cfg],
    #                     condition=IfCondition(_is(mode, 'sim')))

    # --- hardware bring-up (adjust package/executable to your tree) ---
    estimator = Node(package='tt_localization', executable='estimator',
                     name='estimator', output='screen',
                     condition=IfCondition(_is(mode, 'hardware')))

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description="sim | hardware"),
        mpc, manager, viz,
        # sim_plant,        # sim_obstacle,
        # estimator,
    ])


def _is(cfg, value):
    # PythonExpression-free equality check for launch conditions
    from launch.substitutions import PythonExpression
    return PythonExpression(["'", cfg, "' == '", value, "'"])