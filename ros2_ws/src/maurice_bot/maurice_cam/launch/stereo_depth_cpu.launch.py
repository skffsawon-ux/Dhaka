#!/usr/bin/env python3
"""
Launch file for the CPU-based stereo depth estimator (OpenCV StereoSGBM).

This is a simpler, CPU-only implementation for comparison with the VPI-based GPU version.

Usage:
    ros2 launch maurice_cam stereo_depth_cpu.launch.py
    
Compare with GPU version:
    # Terminal 1: Run GPU version
    ros2 launch maurice_cam stereo_depth_estimator.launch.py
    
    # Terminal 2: Run CPU version (publishes to different topics)
    ros2 launch maurice_cam stereo_depth_cpu.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    data_directory_arg = DeclareLaunchArgument(
        'data_directory',
        default_value='/home/jetson1/innate-os/data',
        description='Data directory containing calibration_config_* folder'
    )
    
    stereo_topic_arg = DeclareLaunchArgument(
        'stereo_topic',
        default_value='/mars/main_camera/stereo',
        description='Input stereo image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/mars/main_camera/depth_cpu',
        description='Output depth image topic (16UC1 in mm)'
    )
    
    disparity_topic_arg = DeclareLaunchArgument(
        'disparity_topic',
        default_value='/mars/main_camera/disparity_cpu',
        description='Output disparity visualization topic'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_optical_frame',
        description='TF frame ID'
    )
    
    max_disparity_arg = DeclareLaunchArgument(
        'max_disparity',
        default_value='64',
        description='Maximum disparity'
    )
    
    block_size_arg = DeclareLaunchArgument(
        'block_size',
        default_value='5',
        description='SGBM block size (odd number, 3-11)'
    )
    
    publish_disparity_arg = DeclareLaunchArgument(
        'publish_disparity',
        default_value='true',
        description='Publish disparity visualization'
    )
    
    process_every_n_frames_arg = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='5',
        description='Process 1 out of every N frames'
    )
    
    stereo_width_arg = DeclareLaunchArgument(
        'stereo_width',
        default_value='1280',
        description='Stereo image width (left+right combined)'
    )
    
    stereo_height_arg = DeclareLaunchArgument(
        'stereo_height',
        default_value='480',
        description='Stereo image height'
    )

    # Node
    stereo_depth_cpu_node = Node(
        package='maurice_cam',
        executable='stereo_depth_cpu',
        name='stereo_depth_cpu',
        parameters=[{
            'data_directory': LaunchConfiguration('data_directory'),
            'stereo_topic': LaunchConfiguration('stereo_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'disparity_topic': LaunchConfiguration('disparity_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'max_disparity': LaunchConfiguration('max_disparity'),
            'block_size': LaunchConfiguration('block_size'),
            'publish_disparity': LaunchConfiguration('publish_disparity'),
            'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
            'stereo_width': LaunchConfiguration('stereo_width'),
            'stereo_height': LaunchConfiguration('stereo_height'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        data_directory_arg,
        stereo_topic_arg,
        depth_topic_arg,
        disparity_topic_arg,
        frame_id_arg,
        max_disparity_arg,
        block_size_arg,
        publish_disparity_arg,
        process_every_n_frames_arg,
        stereo_width_arg,
        stereo_height_arg,
        stereo_depth_cpu_node,
    ])

