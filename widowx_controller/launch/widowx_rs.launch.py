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

    color_profile_arg = DeclareLaunchArgument(
        "color_profile_camera1", default_value="640x480x30", description="RealSense colour stream WxHxFPS"
    )
    depth_profile_arg = DeclareLaunchArgument(
        "depth_profile_camera1", default_value="640x480x30", description="RealSense depth stream WxHxFPS"
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
            "load_configs": "false",
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
        condition=IfCondition(LaunchConfiguration("realsense")),
        actions=[
            PushRosNamespace(LaunchConfiguration("camera1")),
            GroupAction(
                scoped=True,
                forwarding=False,
                launch_configurations={
                    "camera_name": LaunchConfiguration("tf_prefix_camera1"),
                    "initial_reset": LaunchConfiguration("initial_reset"),
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile_camera1"),
                    "depth_module.depth_profile": LaunchConfiguration("depth_profile_camera1"),
                },
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [os.path.join(realsense2_camera_dir, "launch", "rs_launch.py")]
                        ),
                    ),
                ],
            ),
        ],
    )

    launch_description_entities = [
        video_stream_provider_arg,
        camera_connector_chart_arg,
        python_node_arg,
        realsense_arg,
        camera1_arg,
        tf_prefix_camera1_arg,
        initial_reset_arg,
        color_profile_arg,
        depth_profile_arg,
        xsarm_control_launch,
        multicam_launch,
        realsense_launch,
    ]

    return LaunchDescription(launch_description_entities)
