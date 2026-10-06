"""Show the RealSense RGB camera feed. Press q or Esc to quit."""

import sys

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    window = "RealSense RGB - q / Esc to quit"
    started = False

    try:
        profile = pipeline.start(config)
        started = True
        print(f"Connected: {profile.get_device().get_info(rs.camera_info.name)}")
        print("Press q or Esc in the video window to quit.")
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if not color:
                continue

            cv2.imshow(window, np.asanyarray(color.get_data()))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, cv2.error) as exc:
        print(f"RealSense viewer: {exc}", file=sys.stderr)
        sys.exit(1)
