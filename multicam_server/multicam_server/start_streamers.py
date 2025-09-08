#!/usr/bin/env python3

import os
import yaml
import re
import shutil
import subprocess
import rclpy
from rclpy.node import Node


class StreamerLauncher(Node):
    def __init__(self):
        super().__init__('start_streamers')

        self.declare_parameter('video_stream_provider', "[1, 2, 3, 4]")
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('retry_on_fail', False)
        self.declare_parameter('camera_connector_chart', '')
        self.declare_parameter('buffer_queue_size', 1)
        self.declare_parameter('python_node', False)

        self.processes = []
        self.start_streamers()

    def get_param(self, name):
        return self.get_parameter(name).value

    def load_connector_chart(self, config_path):
        if not os.path.exists(config_path):
            self.get_logger().error(
                f"The usb connector chart in path {config_path} does not exist."
            )
            self.get_logger().error(
                "Run `v4l2-ctl --list-devices` to find available webcams."
            )
            raise FileNotFoundError(config_path)
        return yaml.load(open(config_path, 'r'), Loader=yaml.CLoader)

    def get_dev(self, output_string, usb_id):
        lines = output_string.decode().split('\n')
        for i, line in enumerate(lines):
            if usb_id in line:
                return re.search('video(\\d+)', lines[i + 1]).group(1)
        raise ValueError(f'usb_id {usb_id} not found!')

    def process_camera_connector_chart(self, config_path):
        connector_chart_dict = self.load_connector_chart(config_path)
        res = subprocess.run(['v4l2-ctl', '--list-devices'], stdout=subprocess.PIPE)
        output_string = res.stdout
        providers, topic_names = [], []
        for topic_name, usb_id in connector_chart_dict.items():
            dev_number = self.get_dev(output_string, usb_id)
            providers.append(dev_number)
            topic_names.append(topic_name)
        return providers, topic_names

    def populate_params(self):
        return {
            'fps': self.get_param('fps'),
            'frame_id': self.get_param('frame_id'),
            'retry_on_fail': self.get_param('retry_on_fail'),
            'buffer_queue_size': self.get_param('buffer_queue_size'),
            'python_node': self.get_param('python_node'),
        }

    def start_streamers(self):
        base_call = ["ros2", "launch", "multicam_server", "streamer.launch.py"]

        topic_names = []
        video_stream_providers = []

        if self.get_param('camera_connector_chart'):
            config_path = self.get_param('camera_connector_chart')
            video_stream_providers, topic_names = self.process_camera_connector_chart(config_path)
        else:
            video_stream_provider = self.get_param('video_stream_provider')
            parsed = eval(video_stream_provider)
            if isinstance(parsed, list):
                video_stream_providers = parsed
            elif isinstance(parsed, int):
                video_stream_providers = [parsed]
            else:
                self.get_logger().error(
                    "Pass either list or integer as video_stream_provider."
                )
            for i in range(len(video_stream_providers)):
                topic_names.append(f'camera{i}')

        for index, (provider, topic_name) in enumerate(zip(video_stream_providers, topic_names)):
            full_params = {
                'video_stream_provider': provider,
                'camera_name': topic_name,
                'node_name': f'streamer_{index}',
            }
            full_params.update(self.populate_params())
            args = [f'{k}:={v}' for k, v in full_params.items()]
            proc = subprocess.Popen(base_call + args)
            self.processes.append(proc)
            self.get_logger().info(f"Started streamer {index} on provider {provider} → topic {topic_name}")

    def shutdown(self):
        for proc in self.processes:
            proc.kill()
            proc.communicate()
        self.get_logger().info("All streamer processes terminated.")


def main(args=None):
    rclpy.init(args=args)
    node = StreamerLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
