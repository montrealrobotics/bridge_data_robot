#!/usr/bin/env python3
"""Pick the cameras to record, with live previews.

Shows every image topic that is currently being published (colour and 16-bit depth) as a thumbnail.
Click a thumbnail to select / deselect it, then press  s  to save the selection to cameras.yaml
(see widowx_envs/utils/camera_config.py), which data collection, teleop and eval read.
Keys:  s save   r rescan topics   q / Esc quit
Run it where the cameras are visible to ROS (inside the robot container, from an ssh -X session).
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from widowx_envs.utils.camera_config import config_path, load_camera_config, save_camera_config

THUMB_W, THUMB_H = 320, 240
WINDOW = "select cameras  (click: toggle   s: save   r: rescan   q: quit)"
DEPTH_ENCODINGS = ("16UC1", "mono16", "32FC1")


class CameraPreview(Node):
    def __init__(self):
        super().__init__("select_cameras")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {}    # topic -> (thumbnail BGR, is_depth)
        self.stamps = {}    # topic -> recent receive times, for the rate display
        self.subs = {}

    def scan(self):
        """Subscribe to every published sensor_msgs/Image topic not subscribed yet."""
        for topic, types in self.get_topic_names_and_types():
            if "sensor_msgs/msg/Image" in types and topic not in self.subs and self.count_publishers(topic) > 0:
                self.subs[topic] = self.create_subscription(
                    Image, topic, lambda msg, t=topic: self._on_image(t, msg), qos_profile_sensor_data)
        return sorted(self.subs)

    def _on_image(self, topic, msg):
        now = time.time()
        with self.lock:
            stamps = [s for s in self.stamps.get(topic, []) if now - s < 2.0] + [now]
            self.stamps[topic] = stamps
            if len(stamps) > 1 and now - stamps[-2] < 0.15:
                return  # a few thumbnails per second is plenty
        is_depth = msg.encoding in DEPTH_ENCODINGS
        try:
            if is_depth:
                d = self.bridge.imgmsg_to_cv2(msg).astype(np.float32)
                valid = d[d > 0]
                hi = np.percentile(valid, 99) if valid.size else 1.0
                img = cv2.applyColorMap(np.clip(d / max(hi, 1e-6) * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
            else:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:  # unsupported encoding etc.
            img = np.zeros((THUMB_H, THUMB_W, 3), np.uint8)
            cv2.putText(img, f"cannot show: {msg.encoding}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            self.get_logger().warn(f"{topic}: {e}", throttle_duration_sec=10.0)
        thumb = cv2.resize(img, (THUMB_W, THUMB_H), interpolation=cv2.INTER_AREA)
        with self.lock:
            self.frames[topic] = (thumb, is_depth)

    def rate(self, topic):
        with self.lock:
            s = self.stamps.get(topic, [])
        return (len(s) - 1) / (s[-1] - s[0]) if len(s) > 1 and s[-1] > s[0] else 0.0


def draw_grid(node, topics, selected, cols):
    rows = max(1, math.ceil(len(topics) / cols))
    canvas = np.full((rows * (THUMB_H + 30), cols * THUMB_W, 3), 40, np.uint8)
    with node.lock:
        frames = dict(node.frames)
    for i, topic in enumerate(topics):
        r, c = divmod(i, cols)
        x, y = c * THUMB_W, r * (THUMB_H + 30)
        thumb, is_depth = frames.get(topic, (None, False))
        if thumb is not None:
            canvas[y:y + THUMB_H, x:x + THUMB_W] = thumb
        else:
            cv2.putText(canvas, "waiting for images...", (x + 70, y + THUMB_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        on = topic in selected
        cv2.rectangle(canvas, (x + 2, y + 2), (x + THUMB_W - 3, y + THUMB_H - 3), (0, 220, 0) if on else (90, 90, 90),
                      4 if on else 1)
        label = f"{'[x]' if on else '[ ]'} {'DEPTH ' if is_depth else ''}{node.rate(topic):4.1f} Hz"
        cv2.putText(canvas, label, (x + 6, y + THUMB_H + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 220, 0) if on else (220, 220, 220), 1)
        cv2.putText(canvas, topic[-48:], (x + 6, y + THUMB_H + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1)
    return canvas


def main():
    rclpy.init()
    node = CameraPreview()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    cfg = load_camera_config() or {"camera_topics": [], "depth_camera_topics": []}
    selected = set(cfg["camera_topics"]) | set(cfg["depth_camera_topics"])
    print(f"camera config: {config_path()}  (currently selected: {sorted(selected) or 'none'})")

    time.sleep(1.0)  # let discovery find the publishers
    topics = node.scan()
    print("image topics found:", topics or "none - are the cameras running?")
    layout = {"topics": topics, "cols": max(1, min(3, len(topics)))}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        i = (y // (THUMB_H + 30)) * layout["cols"] + x // THUMB_W
        if 0 <= i < len(layout["topics"]) and x < layout["cols"] * THUMB_W:
            t = layout["topics"][i]
            selected.symmetric_difference_update({t})

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)
    try:
        while True:
            cv2.imshow(WINDOW, draw_grid(node, layout["topics"], selected, layout["cols"]))
            key = cv2.waitKey(50) & 0xFF
            if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("r"):
                layout["topics"] = node.scan()
                layout["cols"] = max(1, min(3, len(layout["topics"])))
                print("image topics found:", layout["topics"])
            if key == ord("s"):
                with node.lock:
                    depth_flags = {t: f[1] for t, f in node.frames.items()}
                chosen = [t for t in layout["topics"] if t in selected]
                color = [t for t in chosen if not depth_flags.get(t, False)]
                depth = [t for t in chosen if depth_flags.get(t, False)]
                path = save_camera_config(color, depth)
                print(f"saved to {path}\n  camera_topics: {color}\n  depth_camera_topics: {depth}")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
