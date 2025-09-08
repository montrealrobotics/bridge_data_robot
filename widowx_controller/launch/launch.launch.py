#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory


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

    realsense_arg = DeclareLaunchArgument(
        "realsense", default_value="true", description="Enable RealSense camera"
    )

    interbotix_xsarm_control_dir = get_package_share_directory(
        "interbotix_xsarm_control"
    )
    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    interbotix_xsarm_control_dir, "launch", "xsarm_control.launch.py"
                )
            ]
        ),
        launch_arguments={
            "robot_model": EnvironmentVariable("ROBONETV2_ARM"),
            "use_rviz": "False",
        }.items(),
    )

    multicam_server_dir = get_package_share_directory("multicam_server")
    multicam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(multicam_server_dir, "launch", "cameras.launch.py")]
        ),
        launch_arguments={
            "video_stream_provider": LaunchConfiguration("video_stream_provider"),
            "camera_connector_chart": LaunchConfiguration("camera_connector_chart"),
        }.items(),
    )

    launch_description_entities = [
        video_stream_provider_arg,
        camera_connector_chart_arg,
        python_node_arg,
        realsense_arg,
        xsarm_control_launch,
        multicam_launch,
    ]

    return LaunchDescription(launch_description_entities)
