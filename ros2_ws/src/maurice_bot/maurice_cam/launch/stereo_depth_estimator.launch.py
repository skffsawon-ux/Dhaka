#!/usr/bin/env python3
"""
Launch file for the stereo depth estimator node.

This launch file runs the stereo depth estimator as a standalone node or as a
composable node with intra-process communication enabled.

Usage:
    # Standalone node (subscribes to existing stereo topic)
    ros2 launch maurice_cam stereo_depth_estimator.launch.py

    # With custom parameters
    ros2 launch maurice_cam stereo_depth_estimator.launch.py max_disparity:=128 publish_disparity:=true

    # With composable mode for zero-copy IPC (when main camera is in same container)
    ros2 launch maurice_cam stereo_depth_estimator.launch.py use_composable:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments
    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='false',
        description='Run as composable node in a container (enables zero-copy IPC)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    data_directory_arg = DeclareLaunchArgument(
        'data_directory',
        default_value='/home/jetson1/innate-os/data',
        description='Data directory containing calibration_config_* folder'
    )
    
    stereo_topic_arg = DeclareLaunchArgument(
        'stereo_topic',
        default_value='/mars/main_camera/stereo',
        description='Input stereo image topic (side-by-side left+right)'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/mars/main_camera/depth',
        description='Output depth image topic (32FC1 in meters)'
    )
    
    disparity_topic_arg = DeclareLaunchArgument(
        'disparity_topic',
        default_value='/mars/main_camera/disparity',
        description='Output disparity visualization topic (mono8)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_optical_frame',
        description='TF frame ID for output messages'
    )
    
    max_disparity_arg = DeclareLaunchArgument(
        'max_disparity',
        default_value='64',
        description='Maximum disparity for stereo matching (higher = closer objects, more compute)'
    )
    
    publish_disparity_arg = DeclareLaunchArgument(
        'publish_disparity',
        default_value='false',
        description='Publish disparity visualization image'
    )
    
    stereo_width_arg = DeclareLaunchArgument(
        'stereo_width',
        default_value='1280',
        description='Expected stereo image width (full stereo frame, left+right combined)'
    )
    
    stereo_height_arg = DeclareLaunchArgument(
        'stereo_height',
        default_value='480',
        description='Expected stereo image height'
    )
    
    process_every_n_frames_arg = DeclareLaunchArgument(
        'process_every_n_frames',
        default_value='5',
        description='Process 1 out of every N frames (1=all, 2=half rate, 5=6Hz from 30Hz, etc.)'
    )
    
    publish_pointcloud_arg = DeclareLaunchArgument(
        'publish_pointcloud',
        default_value='true',
        description='Publish 3D point cloud from depth'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/mars/main_camera/points',
        description='Point cloud output topic'
    )
    
    pointcloud_decimation_arg = DeclareLaunchArgument(
        'pointcloud_decimation',
        default_value='8',
        description='Point cloud decimation (1=full, 2=half, 4=quarter, 8=1/8th resolution)'
    )
    
    # Disparity filtering arguments
    enable_disparity_filter_arg = DeclareLaunchArgument(
        'enable_disparity_filter',
        default_value='true',
        description='Enable disparity filtering (median + bilateral)'
    )
    
    median_filter_size_arg = DeclareLaunchArgument(
        'median_filter_size',
        default_value='5',
        description='Median filter kernel size (0=disabled, 3/5/7). Removes outlier speckles.'
    )
    
    bilateral_filter_size_arg = DeclareLaunchArgument(
        'bilateral_filter_size',
        default_value='5',
        description='Bilateral filter kernel size (0=disabled, 3/5/7/9). Edge-preserving smoothing.'
    )
    
    bilateral_sigma_space_arg = DeclareLaunchArgument(
        'bilateral_sigma_space',
        default_value='1.5',
        description='Bilateral filter spatial sigma (larger = more spatial smoothing)'
    )
    
    bilateral_sigma_color_arg = DeclareLaunchArgument(
        'bilateral_sigma_color',
        default_value='50.0',
        description='Bilateral filter intensity sigma (larger = smoother across intensity boundaries)'
    )

    # Common parameters for both modes
    node_parameters = [{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'data_directory': LaunchConfiguration('data_directory'),
        'stereo_topic': LaunchConfiguration('stereo_topic'),
        'depth_topic': LaunchConfiguration('depth_topic'),
        'disparity_topic': LaunchConfiguration('disparity_topic'),
        'frame_id': LaunchConfiguration('frame_id'),
        'max_disparity': LaunchConfiguration('max_disparity'),
        'publish_disparity': LaunchConfiguration('publish_disparity'),
        'stereo_width': LaunchConfiguration('stereo_width'),
        'stereo_height': LaunchConfiguration('stereo_height'),
        'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
        'publish_pointcloud': LaunchConfiguration('publish_pointcloud'),
        'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
        'pointcloud_decimation': LaunchConfiguration('pointcloud_decimation'),
        'enable_disparity_filter': LaunchConfiguration('enable_disparity_filter'),
        'median_filter_size': LaunchConfiguration('median_filter_size'),
        'bilateral_filter_size': LaunchConfiguration('bilateral_filter_size'),
        'bilateral_sigma_space': LaunchConfiguration('bilateral_sigma_space'),
        'bilateral_sigma_color': LaunchConfiguration('bilateral_sigma_color'),
    }]

    # Standalone node (default)
    standalone_node = Node(
        package='maurice_cam',
        executable='stereo_depth_estimator',
        name='stereo_depth_estimator',
        parameters=node_parameters,
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('use_composable')),
    )

    # Composable node in container (for zero-copy IPC)
    composable_container = ComposableNodeContainer(
        name='depth_estimator_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='maurice_cam',
                plugin='maurice_cam::StereoDepthEstimator',
                name='stereo_depth_estimator',
                parameters=node_parameters,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_composable')),
    )

    return LaunchDescription([
        # Launch arguments
        use_composable_arg,
        use_sim_time_arg,
        data_directory_arg,
        stereo_topic_arg,
        depth_topic_arg,
        disparity_topic_arg,
        frame_id_arg,
        max_disparity_arg,
        publish_disparity_arg,
        stereo_width_arg,
        stereo_height_arg,
        process_every_n_frames_arg,
        publish_pointcloud_arg,
        pointcloud_topic_arg,
        pointcloud_decimation_arg,
        enable_disparity_filter_arg,
        median_filter_size_arg,
        bilateral_filter_size_arg,
        bilateral_sigma_space_arg,
        bilateral_sigma_color_arg,
        # Nodes (one will be active based on use_composable)
        standalone_node,
        composable_container,
    ])

