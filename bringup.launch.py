# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

# Fonksiyon tanımı (girinti yok)
def generate_launch_description(): 

    # --- Değişken tanımlamaları (4 normal boşluk girinti) ---
    pkg_path = get_package_share_directory('my_robot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    # Xacro'yu işlemeden önce varlığını kontrol et (ekstra güvenlik)
    if not os.path.exists(xacro_file):
        raise Exception(f"URDF file not found: {xacro_file}")
    try:
        robot_description_xml = xacro.process_file(xacro_file).toxml()
    except Exception as e:
        raise Exception(f"Error processing XACRO file: {e}")


    world_file = os.path.join(pkg_path, 'worlds', 'ball_world.sdf')
    if not os.path.exists(world_file):
        raise Exception(f"World file not found: {world_file}")

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'config.rviz')
    if not os.path.exists(rviz_config_file):
        raise Exception(f"RViz config file not found: {rviz_config_file}")

    # --- Düğüm ve Süreç Tanımlamaları (4 normal boşluk girinti) ---

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot'
        ],
        output='screen'
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True}]
    )

    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # E. RViz'i Başlat (GECİKMESİZ - NORMAL HALİ)
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    start_ball_chaser = Node(
        package='my_robot_bringup',
        executable='ball_chaser',
        name='ball_chaser',
        output='screen',
    )

    start_base_footprint_to_base_link_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'] # Düzeltildi
    )

    # --- Return İfadesi (4 normal boşluk girinti, fonksiyonun İÇİNDE) ---
    return LaunchDescription([
        start_gazebo,
        spawn_entity,
        start_robot_state_publisher,
        start_joint_state_publisher,
        start_rviz, # <-- GECİKMESİZ HALİNE DÖNDÜ
        start_ball_chaser,
        start_base_footprint_to_base_link_tf
    ])
# Fonksiyon tanımı burada bitti (girinti yok)
