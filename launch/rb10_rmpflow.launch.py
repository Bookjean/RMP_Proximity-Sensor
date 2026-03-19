#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('rb10_rmpflow_rviz')

    # Get URDF path
    urdf_path = os.path.join(pkg_share, 'urdf', 'rb10_1300e.urdf')

    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    params_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )

    use_interactive_goal_arg = DeclareLaunchArgument(
        'use_interactive_goal',
        default_value='true',
        description='Start the interactive goal publisher'
    )

    use_obstacles_arg = DeclareLaunchArgument(
        'use_obstacles',
        default_value='true',
        description='Enable obstacle manager'
    )

    use_proximity_bridge_arg = DeclareLaunchArgument(
        'use_proximity_bridge',
        default_value='false',
        description='Use external proximity_distance topics to build obstacle markers'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 100.0,
        }]
    )

    rmpflow_controller = Node(
        package='rb10_rmpflow_rviz',
        executable='rmpflow_controller',
        name='rmpflow_controller',
        output='screen',
        parameters=[params_path],
    )

    interactive_goal = Node(
        package='rb10_rmpflow_rviz',
        executable='interactive_goal',
        name='interactive_goal',
        output='screen',
        parameters=[params_path],
        condition=IfCondition(LaunchConfiguration('use_interactive_goal')),
    )

    # Obstacle Manager
    obstacle_manager = Node(
        package='rb10_rmpflow_rviz',
        executable='obstacle_manager',
        name='obstacle_manager',
        output='screen',
        condition=IfCondition(PythonExpression([
            '"', LaunchConfiguration('use_obstacles'), '" == "true" and "',
            LaunchConfiguration('use_proximity_bridge'), '" != "true"',
        ])),
        parameters=[params_path],
    )

    proximity_obstacle_bridge = Node(
        package='rb10_rmpflow_rviz',
        executable='proximity_obstacle_bridge',
        name='proximity_obstacle_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_proximity_bridge')),
        parameters=[params_path],
    )

    tof_ray_visualizer = Node(
        package='rb10_rmpflow_rviz',
        executable='tof_ray_visualizer',
        name='tof_ray_visualizer',
        output='screen',
        parameters=[{
            'publish_rate': 20.0,
            'max_range': 0.2,
            'min_range': 0.02,
            'sensor_face_width': 0.25,
            'sensor_face_height': 0.25,
            'sensor_grid_resolution': 7,
            'edge_range_ratio': 0.6,
            'edge_falloff_power': 2.0,
        }],
    )

    # RViz
    rviz_config_path = os.path.join(pkg_share, 'config', 'rb10_rmpflow.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    )

    return LaunchDescription([
        use_rviz_arg,
        use_interactive_goal_arg,
        use_obstacles_arg,
        use_proximity_bridge_arg,
        robot_state_publisher,
        rmpflow_controller,
        interactive_goal,
        obstacle_manager,
        proximity_obstacle_bridge,
        tof_ray_visualizer,
        rviz,
    ])
