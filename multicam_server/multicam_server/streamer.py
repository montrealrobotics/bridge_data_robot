#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Streamer(Node):
    def __init__(self):
        super().__init__("streamer")

        self.declare_parameter("fps", 30)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("retry_on_fail", False)
        self.declare_parameter("buffer_queue_size", 1)
        self.declare_parameter("camera_name", "camera0")
        self.declare_parameter("video_stream_provider", "[1, 2, 3, 4]")

        self._fps = self.get_parameter("fps").value
        self._frame_id = self.get_parameter("frame_id").value
        self._retry_on_fail = self.get_parameter("retry_on_fail").value
        self._buffer_queue_size = self.get_parameter("buffer_queue_size").value
        self._topic_name = self.get_parameter("camera_name").value
        self._video_stream_provider = str(
            self.get_parameter("video_stream_provider").value
        )

        self.full_resource_path = "/dev/video" + str(self._video_stream_provider)
        success = self.setup_capture_device()
        if not success:
            self.get_logger().error("Failed to open capture device. Exiting.")
            return

        self.publisher = self.create_publisher(
            Image, self._topic_name + "/image_raw", 10
        )
        self.bridge = CvBridge()
        self._buffer = []

        self.capture_timer = self.create_timer(0.0, self.capture)

        self.publish_timer = self.create_timer(
            1.0 / float(self._fps), self.publish_frame
        )

    def setup_capture_device(self):
        if not os.path.exists(self.full_resource_path):
            self.get_logger().error(
                f"Device '{self.full_resource_path}' does not exist."
            )
            return False

        self.get_logger().info(
            f"Trying to open resource: '{self.full_resource_path}' for topic '{self._topic_name}'"
        )
        self.cap = cv2.VideoCapture(self.full_resource_path)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"Error opening resource: {self.full_resource_path}"
            )
            self.get_logger().info("The device may already be in use.")
            return False

        self.get_logger().info(f"Correctly opened resource {self.full_resource_path}.")
        return True

    def capture(self):
        rval, frame = self.cap.read()
        if not rval:
            self.get_logger().warn(
                f"Frame not captured from {self.full_resource_path}."
            )
            if self._retry_on_fail:
                self.get_logger().info(f"Retrying device {self.full_resource_path}...")
                self.setup_capture_device()
            return

        reading = [frame, self.get_clock().now().to_msg()]
        if len(self._buffer) >= self._buffer_queue_size:
            self._buffer.pop(0)
        self._buffer.append(reading)

    def publish_frame(self):
        if not self._buffer:
            return

        frame, timestamp = self._buffer.pop(0)
        imgmsg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        imgmsg.header.frame_id = self._frame_id
        imgmsg.header.stamp = timestamp
        self.publisher.publish(imgmsg)


def main(args=None):
    rclpy.init(args=args)
    node = Streamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "cap") and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
