#! /usr/bin/env python

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    video_stream_provider_arg = DeclareLaunchArgument(
        'video_stream_provider',
        default_value='0',
        description='Video stream provider parameter'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='FPS parameter'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame id parameter'
    )

    retry_on_fail_arg = DeclareLaunchArgument(
        'retry_on_fail',
        default_value='false',
        description='Retry on fail parameter'
    )

    buffer_queue_size_arg = DeclareLaunchArgument(
        'buffer_queue_size',
        default_value='2',
        description='Buffer queue size parameter'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera0',
        description='Camera name parameter'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='streamer',
        description='Node name parameter'
    )

    python_node_arg = DeclareLaunchArgument(
        'python_node',
        default_value='false',
        description='Use Python node'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    device_0_arg = DeclareLaunchArgument(
        'device_0',
        default_value='0',
        description='Device 0 parameter'
    )

    python_node = Node(
        condition=IfCondition(LaunchConfiguration('python_node')),
        package='multicam_server',
        executable='streamer.py',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            {'video_stream_provider': LaunchConfiguration('video_stream_provider')},
            {'fps': LaunchConfiguration('fps')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'retry_on_fail': LaunchConfiguration('retry_on_fail')},
            {'buffer_queue_size': LaunchConfiguration('buffer_queue_size')},
            {'camera_name': LaunchConfiguration('camera_name')},
        ]
    )

    non_python_node_launch = Node(
            condition=UnlessCondition(LaunchConfiguration('python_node')),
            package='image_publisher', executable='image_publisher_node', output='screen',
            arguments=[LaunchConfiguration('device_0')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('image_raw', '/camera/image_raw'),
                        ('camera_info', '/camera/camera_info')
                        ]
    )

    return LaunchDescription([
        python_node,
        non_python_node_launch,
        video_stream_provider_arg,
        fps_arg,
        frame_id_arg,
        buffer_queue_size_arg,
        retry_on_fail_arg,
        camera_name_arg,
        node_name_arg,
        python_node_arg,
        sim_time_arg,
        device_0_arg
    ])