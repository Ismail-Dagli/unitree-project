"""
g1_autonomy.launch.py — Master launch file for the Unitree G1 autonomy stack.

Brings up all components in the correct order:
  1. g1_tf_bridge        — odometry TF + LiDAR static TF + scan conversion
  2. pointcloud_to_laserscan (optional, more robust than the built-in converter)
  3. slam_toolbox         — online async SLAM
  4. Nav2 bringup         — full navigation stack
  5. RViz2                — visualisation with pre-configured layout
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_autonomy_pkg')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'g1_nav.rviz')

    # ---- Launch arguments -------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_pointcloud_to_laserscan = LaunchConfiguration('use_pc2_to_scan')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulated clock')

    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Launch RViz2 with pre-configured layout')

    declare_pc2_to_scan = DeclareLaunchArgument(
        'use_pc2_to_scan', default_value='false',
        description='Use pointcloud_to_laserscan node (bridge has inline converter)')

    # ---- 1. G1 TF Bridge -------------------------------------------------------
    g1_tf_bridge_node = Node(
        package='g1_autonomy_pkg',
        executable='g1_tf_bridge',
        name='g1_tf_bridge',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'cloud_topic': '/utlidar/cloud',
            'scan_topic': '/scan_bridge',    # internal; may be overridden by pc2_to_scan
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'lidar_frame': 'lidar_link',
            'lidar_x': 0.05,
            'lidar_y': 0.0,
            'lidar_z': 1.1,
            'use_sim_time': use_sim_time,
        }],
    )

    # ---- 2. PointCloud2 → LaserScan (robust, GPU-accelerated if available) ------
    pc2_to_scan_node = Node(
        condition=IfCondition(use_pointcloud_to_laserscan),
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'lidar_link',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,   # ~0.25°
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 12.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', '/utlidar/cloud'),
            ('scan', '/scan'),
        ],
    )

    # ---- 3. SLAM Toolbox --------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/scan', '/scan'),
        ],
    )

    # ---- 4. Nav2 Stack -----------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    # ---- 5. RViz2 ----------------------------------------------------------------
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- Assemble ----------------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_rviz,
        declare_pc2_to_scan,

        g1_tf_bridge_node,
        pc2_to_scan_node,
        slam_node,
        nav2_launch,
        rviz_node,
    ])
