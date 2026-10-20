"""RealSense RGB capture and stream options; construction does not open hardware."""

import argparse
import time

import numpy as np

from io_utils import utc_now


def positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def add_camera_arguments(parser, width=1280, height=720):
    parser.add_argument("--serial", help="RealSense serial number")
    parser.add_argument("--width", type=positive_int, default=width)
    parser.add_argument("--height", type=positive_int, default=height)
    parser.add_argument("--fps", type=positive_int, default=30)


def require_same_camera(actual, expected):
    for key in ("serial", "image_size"):
        if actual[key] != expected[key]:
            raise ValueError(f"Camera {key} differs from the saved calibration")


class RealSenseCamera:
    def __init__(self, args):
        self.args = args
        self.pipeline = None

    def __enter__(self):
        import pyrealsense2 as rs

        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.args.serial:
            config.enable_device(self.args.serial)
        config.enable_stream(rs.stream.color, self.args.width, self.args.height,
                             rs.format.bgr8, self.args.fps)
        started = False
        try:
            profile = self.pipeline.start(config)
            started = True
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = stream.get_intrinsics()
            device = profile.get_device()
            self.metadata = {
                "serial": device.get_info(rs.camera_info.serial_number),
                "name": device.get_info(rs.camera_info.name),
                "image_size": [intr.width, intr.height],
                "fps": stream.fps(),
                "matrix": [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
                "dist_coeffs": list(intr.coeffs),
                "distortion_model": str(intr.model).split(".")[-1],
                "source": "realsense_factory_color",
                "frame": "color_optical (x right, y down, z forward)",
            }
            # Warm up exposure and discard startup frames.
            for _ in range(20):
                self.read()
            return self
        except BaseException:
            if started:
                self.pipeline.stop()
            raise

    def read(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        frame = frames.get_color_frame()
        if not frame:
            raise RuntimeError("No RealSense color frame")
        return self.frame_data(frame)

    def poll(self):
        """Nonblocking preview read, so robot stop monitoring never waits for USB."""
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return None
        frame = frames.get_color_frame()
        return self.frame_data(frame) if frame else None

    @staticmethod
    def frame_data(frame):
        return np.asanyarray(frame.get_data()).copy(), {
            "frame_number": frame.get_frame_number(),
            "device_timestamp_ms": frame.get_timestamp(),
            "timestamp_domain": str(frame.get_frame_timestamp_domain()),
            "host_monotonic_s": time.monotonic(),
            "host_utc": utc_now(),
        }

    def fresh(self):
        # All these frames arrive after the stationary checks started.
        for _ in range(3):
            result = self.read()
        return result

    def __exit__(self, *_):
        self.pipeline.stop()
