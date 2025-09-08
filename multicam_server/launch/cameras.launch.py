#!/usr/bin/env python


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    video_stream_provider_arg = DeclareLaunchArgument(
        "video_stream_provider",
        default_value="0",
        description="Video stream provider parameter",
    )

    camera_connector_chart_arg = DeclareLaunchArgument(
        "camera_connector_chart",
        default_value="",
        description="Camera connector chart parameter",
    )

    python_node_arg = DeclareLaunchArgument(
        "python_node", default_value="false", description="Use Python node"
    )

    buffer_queue_size_arg = DeclareLaunchArgument(
        "buffer_queue_size",
        default_value="1",
        description="Buffer queue size parameter",
    )

    fps_arg = DeclareLaunchArgument(
        "fps", default_value="30", description="FPS parameter"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="world", description="Frame id parameter"
    )

    retry_on_fail_arg = DeclareLaunchArgument(
        "retry_on_fail", default_value="false", description="Retry on fail parameter"
    )

    multicam_node = Node(
        package="multicam_server",
        executable="start_streamers.py",
        output="screen",
        parameters=[
            {
                "video_stream_provider": LaunchConfiguration("video_stream_provider"),
                "fps": LaunchConfiguration("fps"),
                "frame_id": LaunchConfiguration("frame_id"),
                "retry_on_fail": LaunchConfiguration("retry_on_fail"),
                "camera_connector_chart": LaunchConfiguration("camera_connector_chart"),
                "buffer_queue_size": LaunchConfiguration("buffer_queue_size"),
                "python_node": LaunchConfiguration("python_node"),
            }
        ],
    )

    return LaunchDescription(
        [
            video_stream_provider_arg,
            camera_connector_chart_arg,
            python_node_arg,
            buffer_queue_size_arg,
            fps_arg,
            frame_id_arg,
            retry_on_fail_arg,
            multicam_node,
        ]
    )
