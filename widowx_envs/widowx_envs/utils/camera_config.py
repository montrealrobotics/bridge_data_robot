"""Shared camera selection, written by select_cameras.py and read by data collection, teleop and eval.

The file is widowx_envs/cameras.yaml (inside the mounted repo, so it persists on the host), or the path
in $WIDOWX_CAMERA_CONFIG:

    camera_topics:          # colour images, saved as images0, images1, ...
      - /D435/camera/D435/color/image_raw
    depth_camera_topics:    # 16-bit depth, saved as depth_images0, ...
      - /D435/camera/D435/depth/image_rect_raw
"""
import os

import yaml

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))),
                            "cameras.yaml")


def config_path():
    return os.environ.get("WIDOWX_CAMERA_CONFIG", DEFAULT_PATH)


def load_camera_config(path=None):
    """Return {'camera_topics': [...], 'depth_camera_topics': [...]}, or None if there is no config file."""
    path = path or config_path()
    if not os.path.exists(path):
        return None
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    return {
        "camera_topics": list(cfg.get("camera_topics") or []),
        "depth_camera_topics": list(cfg.get("depth_camera_topics") or []),
    }


def save_camera_config(camera_topics, depth_camera_topics, path=None):
    path = path or config_path()
    with open(path, "w") as f:
        f.write("# written by select_cameras.py\n")
        yaml.safe_dump({"camera_topics": list(camera_topics), "depth_camera_topics": list(depth_camera_topics)},
                       f, default_flow_style=False, sort_keys=False)
    return path


def imtopics(default_camera_topics=(), default_depth_topics=()):
    """IMTopic lists for env_params (camera_topics, depth_camera_topics), falling back to the defaults."""
    from multicam_server.topic_utils import IMTopic
    cfg = load_camera_config()
    color = cfg["camera_topics"] if cfg else list(default_camera_topics)
    depth = cfg["depth_camera_topics"] if cfg else list(default_depth_topics)
    return [IMTopic(t) for t in color], [IMTopic(t, dtype="16UC1") for t in depth]
