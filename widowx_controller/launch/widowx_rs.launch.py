#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    video_stream_provider_arg = DeclareLaunchArgument(
        "video_stream_provider",
        default_value="",
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

    serial_no_camera1_arg = DeclareLaunchArgument(
        "serial_no_camera1",
        default_value="_913522070688",
        description="Serial number for camera1 (Replace with actual serial number)",
    )

    camera1_arg = DeclareLaunchArgument(
        "camera1",
        default_value="D435",
        description="Camera1 name (Replace with camera name)",
    )

    tf_prefix_camera1_arg = DeclareLaunchArgument(
        "tf_prefix_camera1",
        default_value=LaunchConfiguration("camera1"),
        description="TF prefix for camera1",
    )

    initial_reset_arg = DeclareLaunchArgument(
        "initial_reset", default_value="true", description="Initial reset parameter"
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
            "use_rviz": "false",
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

    realsense2_camera_dir = get_package_share_directory("realsense2_camera")
    realsense_launch = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("camera1")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(realsense2_camera_dir, "launch", "rs_launch.py")]
                ),
                condition=IfCondition(LaunchConfiguration("realsense")),
                launch_arguments={
                    "serial_no": LaunchConfiguration("serial_no_camera1"),
                    "camera_name": LaunchConfiguration("tf_prefix_camera1"),
                    "initial_reset": LaunchConfiguration("initial_reset"),
                }.items(),
            ),
        ]
    )

    launch_description_entities = [
        video_stream_provider_arg,
        camera_connector_chart_arg,
        python_node_arg,
        realsense_arg,
        serial_no_camera1_arg,
        camera1_arg,
        tf_prefix_camera1_arg,
        initial_reset_arg,
        xsarm_control_launch,
        multicam_launch,
        realsense_launch,
    ]

    return LaunchDescription(launch_description_entities)
