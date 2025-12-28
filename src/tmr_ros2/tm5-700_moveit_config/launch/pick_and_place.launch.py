#!/usr/bin/env python3
"""
ピック＆プレースデモ用launchファイル
MoveIt + RViz + デモスクリプトを起動
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージディレクトリ
    pkg_dir = get_package_share_directory('tm5-700_moveit_config')
    
    # RVizとMoveItを起動
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'demo.launch.py')
        )
    )
    
    # ピック＆プレースデモノード
    pick_and_place_node = Node(
        package='tm5-700_moveit_config',
        executable='pick_and_place_simple.py',
        name='pick_and_place_demo',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        demo_launch,
        pick_and_place_node,
    ])
