#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get absolute path to waypoints
    trajectory_pkg = get_package_share_directory('trajectory_controller')
    waypoint_file = os.path.join(trajectory_pkg, 'config', 'waypoints.yaml')
    
    # Path Smoother Node
    path_smoother = Node(
        package='trajectory_controller',
        executable='path_smoother.py',
        name='path_smoother',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
        }]
    )
    
    # Smoothed Path Visualizer Node
    path_visualizer = Node(
        package='trajectory_controller',
        executable='smoothed_path_visualizer.py',
        name='smoothed_path_visualizer',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
        }]
    )
    
    # Trajectory Generation Node
    trajectory_gen = Node(
        package='trajectory_controller',
        executable='trajectory_generation.py',
        name='trajectory_generation',
        output='screen',
    )
    
    # Improved Obstacle-Aware Pure Pursuit Controller
    obstacle_aware_controller = Node(
        package='trajectory_controller',
        executable='trajectory_controller.py',
        name='trajectory_controller',
        output='screen',
        parameters=[{
            # AMR85 Physical Parameters
            'wheelbase': 0.27,
            'track_width': 0.265,
            
            # Pure Pursuit Parameters - Dynamic adjustment enabled in code
            'lookahead_distance': 0.8,       # Base lookahead (dynamically adjusted 0.3-0.8)
            
            # Velocity Control - Balanced for cylinder navigation
            'target_speed': 0.15,            # Conservative for tight spaces
            'max_linear_velocity': 0.20,     # Moderate max speed
            'min_linear_velocity': 0.05,     # Maintain momentum
            'max_angular_velocity': 1.0,     # Smooth turning
            
            # Obstacle Avoidance Parameters - LESS AGGRESSIVE for expected obstacles
            'obstacle_distance_threshold': 0.10,       # Reduced - only critical obstacles
            'warning_distance_threshold': 0.5,         # Reduced - less early warning
            'side_clearance': 0.25,                    # Tighter side clearance
            'avoidance_gain': 0.5,                     # Very gentle avoidance
            
            # Control Parameters
            'goal_tolerance': 0.15,                # Goal reached threshold
            'path_completion_threshold': 0.95,     # Path completion percentage
            
            # Stuck Detection Parameters (NEW)
            'stuck_velocity_threshold': 0.02,      # Consider stuck if velocity < 0.02 m/s
            'stuck_time_threshold': 3.0,           # Trigger recovery after 3 seconds stuck
        }]
    )
    
    return LaunchDescription([
        path_smoother,
        path_visualizer,
        trajectory_gen,
        obstacle_aware_controller,
    ])