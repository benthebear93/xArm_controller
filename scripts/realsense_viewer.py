"""Preview RealSense RGB/depth, or test reception without a desktop window."""

import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from realsense_camera import add_camera_arguments, positive_int


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_camera_arguments(parser, width=640, height=480)
    parser.add_argument("--depth", action="store_true", help="Also receive and preview the depth stream")
    parser.add_argument("--headless", action="store_true", help="No window; receive 90 frames by default")
    parser.add_argument("--frames", type=positive_int, help="Stop after this many matched frames")
    parser.add_argument("--snapshot", type=Path, help="Save the last RGB frame as PNG/JPEG")
    parser.add_argument("--list-devices", action="store_true", help="List connected cameras and exit")
    args = parser.parse_args(argv)
    if args.snapshot and args.snapshot.suffix.lower() not in (".png", ".jpg", ".jpeg"):
        parser.error("--snapshot must end in .png, .jpg, or .jpeg")

    context = rs.context()
    devices = list(context.query_devices())
    for device in devices:
        print(f"Camera: {device.get_info(rs.camera_info.name)} | "
              f"serial={device.get_info(rs.camera_info.serial_number)}", flush=True)
    if args.list_devices:
        return
    if not devices:
        raise RuntimeError("No RealSense camera found; check USB cable, power, and device permissions")
    if args.serial is None:
        if len(devices) != 1:
            raise RuntimeError("Multiple cameras found; choose one with --serial")
        args.serial = devices[0].get_info(rs.camera_info.serial_number)
    elif args.serial not in [d.get_info(rs.camera_info.serial_number) for d in devices]:
        raise RuntimeError(f"Requested camera {args.serial} is not connected")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    if args.depth:
        config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    window = "RealSense - RGB | Depth (unaligned) - q / Esc to quit" if args.depth else "RealSense RGB - q / Esc to quit"
    frame_limit = args.frames or (90 if args.headless else None)
    started = False
    window_open = False
    rgb_image = depth_preview = None
    received = 0
    previous_number = None
    first_time = last_time = None
    depth_valid_total = 0.0

    try:
        profile = pipeline.start(config)
        started = True
        device = profile.get_device()
        print(f"Connected: {device.get_info(rs.camera_info.name)}", flush=True)
        for key in (rs.camera_info.firmware_version, rs.camera_info.usb_type_descriptor):
            if device.supports(key):
                print(f"  {str(key).split('.')[-1]}: {device.get_info(key)}", flush=True)
        streams = (rs.stream.color, rs.stream.depth) if args.depth else (rs.stream.color,)
        for kind in streams:
            stream = profile.get_stream(kind).as_video_stream_profile()
            print(f"  {kind}: {stream.width()} x {stream.height()} @ {stream.fps()} FPS", flush=True)
        if args.depth:
            print(f"  Depth scale: {device.first_depth_sensor().get_depth_scale()} m/unit", flush=True)
        colorizer = rs.colorizer() if args.depth else None
        if not args.headless:
            print("Press q or Esc in the video window to quit.", flush=True)
            cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
            window_open = True

        # Let auto-exposure settle before counting or saving frames.
        for _ in range(30):
            pipeline.wait_for_frames(timeout_ms=5000)

        last_received = time.monotonic()
        while frame_limit is None or received < frame_limit:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            depth = frames.get_depth_frame() if args.depth else None
            if (not color or (args.depth and not depth)
                    or color.get_frame_number() == previous_number):
                if time.monotonic() - last_received > 5:
                    raise RuntimeError("No new complete RGB/depth frame received for 5 seconds")
                continue
            previous_number = color.get_frame_number()
            last_time = last_received = time.monotonic()
            if first_time is None:
                first_time = last_time
            received += 1
            rgb_image = np.asanyarray(color.get_data()).copy()
            if args.depth:
                raw_depth = np.asanyarray(depth.get_data())
                depth_valid_total += float(np.count_nonzero(raw_depth) / raw_depth.size)
                # The SDK colorizer produces RGB8; OpenCV display/files expect BGR.
                depth_preview = cv2.cvtColor(np.asanyarray(colorizer.colorize(depth).get_data()), cv2.COLOR_RGB2BGR)
            if received == 1 or received % 30 == 0:
                print(f"Received {received} frames; RGB frame #{previous_number}", flush=True)
            if not args.headless:
                view = np.hstack((rgb_image, depth_preview)) if args.depth else rgb_image
                cv2.imshow(window, view)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            if window_open:
                cv2.destroyAllWindows()

    if not received:
        raise RuntimeError("No frames received")
    measured_fps = (received - 1) / (last_time - first_time) if received > 1 else 0
    print(f"Reception OK: {received} RGB frames, measured {measured_fps:.2f} FPS", flush=True)
    if args.depth:
        valid = depth_valid_total / received
        print(f"Depth: {received} frames; mean nonzero coverage {100 * valid:.1f}%", flush=True)
        if valid == 0:
            raise RuntimeError("Depth frames contain only zero/invalid distances")
    if args.snapshot:
        args.snapshot.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.snapshot), rgb_image):
            raise RuntimeError(f"Could not save {args.snapshot}")
        print(f"RGB snapshot: {args.snapshot.resolve()}", flush=True)
        if args.depth:
            depth_path = args.snapshot.with_name(args.snapshot.stem + "_depth.png")
            if not cv2.imwrite(str(depth_path), depth_preview):
                raise RuntimeError(f"Could not save {depth_path}")
            print(f"Depth visualization (not metric depth): {depth_path.resolve()}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, cv2.error) as exc:
        print(f"RealSense viewer: {exc}", file=sys.stderr)
        sys.exit(1)
