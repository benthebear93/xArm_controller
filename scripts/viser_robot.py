"""Show live xArm7 joint feedback in Viser, without sending motion commands.

Open http://localhost:8080. Optional --plan previews saved automatic-calibration
poses on a separate translucent model. --demo works without a robot.
"""

import argparse
from pathlib import Path
import time

import numpy as np

from geometry import (
    finite_array, flange_from_tcp, pose_error, pose_to_transform, transform, transform_to_pose,
)
from io_utils import read_json, write_json
from robot_common import ROBOT_IP, create_arm, read_result, run_cli

MODEL_PATH = Path(__file__).resolve().parents[1] / "assets/robots/xarm7/xarm7_nohand.xml"


class RobotModel:
    def __init__(self):
        import mujoco as mj

        self.mj = mj
        self.model = mj.MjModel.from_xml_path(str(MODEL_PATH))
        self.model.body_pos[self.model.body("link_base").id] = 0
        self.data = mj.MjData(self.model)
        self.joint_addresses = [int(self.model.joint(f"joint{i}").qposadr[0]) for i in range(1, 8)]
        self.site_id = self.model.site("attachment_site").id
        self.handles = []
        self.set_joints([0] * 7)

    def set_joints(self, degrees):
        joints = finite_array(degrees, (7,), "joint feedback")
        self.data.qpos[self.joint_addresses] = np.radians(joints)
        self.mj.mj_forward(self.model, self.data)

    def quat(self, rotation):
        value = np.empty(4)
        self.mj.mju_mat2Quat(value, np.asarray(rotation).reshape(9))
        return value

    def flange(self):
        return transform(self.data.site_xmat[self.site_id].reshape(3, 3), self.data.site_xpos[self.site_id])

    def add_to_scene(self, server, prefix="/robot", ghost=False):
        for geom_id in range(self.model.ngeom):
            if self.model.geom_type[geom_id] != self.mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh_id = int(self.model.geom_dataid[geom_id])
            start, count = self.model.mesh_vertadr[mesh_id], self.model.mesh_vertnum[mesh_id]
            face_start, face_count = self.model.mesh_faceadr[mesh_id], self.model.mesh_facenum[mesh_id]
            handle = server.scene.add_mesh_simple(
                f"{prefix}/mesh_{geom_id}",
                vertices=self.model.mesh_vert[start:start + count].copy(),
                faces=self.model.mesh_face[face_start:face_start + face_count].copy(),
                color=(70, 180, 240) if ghost else (220, 225, 232),
                opacity=0.3 if ghost else 1.0,
            )
            self.handles.append((geom_id, handle))
        self.axes = server.scene.add_frame(f"{prefix}/flange", axes_length=.09, axes_radius=.002)
        self.update_scene()

    def update_scene(self):
        for geom_id, handle in self.handles:
            handle.position = self.data.geom_xpos[geom_id]
            handle.wxyz = self.quat(self.data.geom_xmat[geom_id])
        self.axes.position = self.data.site_xpos[self.site_id]
        self.axes.wxyz = self.quat(self.data.site_xmat[self.site_id])


def read_feedback(arm):
    """Read feedback even while moving or faulted; visualization never clears faults."""
    if not arm.connected:
        raise RuntimeError("Robot connection lost")
    joints = finite_array(read_result(arm.get_servo_angle(is_radian=False, is_real=True),
                                      "Read actual joints"), (7,), "joint angles")
    tcp = finite_array(read_result(arm.get_position(is_radian=False), "Read TCP"), (6,), "TCP pose")
    error, warning = read_result(arm.get_err_warn_code(), "Read robot status")
    return {
        "joints_deg": joints.tolist(), "tcp_pose_mm_deg": tcp.tolist(),
        "tcp_offset_mm_deg": finite_array(arm.tcp_offset, (6,), "TCP offset").tolist(),
        "world_offset_mm_deg": finite_array(arm.world_offset, (6,), "world offset").tolist(),
        "state": read_result(arm.get_state(), "Read robot state"),
        "error": error, "warning": warning, "timestamp_unix": time.time(),
    }


def demo_feedback(model):
    # These are arbitrary demo angles, never actual robot feedback.
    joints = [0, -14, 0, 52, 0, 66, 0]
    model.set_joints(joints)
    flange = model.flange()
    return {"joints_deg": joints, "tcp_pose_mm_deg": transform_to_pose(flange),
            "tcp_offset_mm_deg": [0] * 6, "world_offset_mm_deg": [0] * 6,
            "state": 2, "error": 0, "warning": 0, "timestamp_unix": time.time()}


class PlanTimeline:
    """Approximate joint interpolation; no connection to robot control."""

    def __init__(self, document):
        self.joints = np.asarray([finite_array(t["joints_deg"], (7,), "planned joints")
                                  for t in document["targets"]])
        if not len(self.joints):
            raise ValueError("Motion plan has no target poses")
        speed = float(document["speed_deg_s"])
        acceleration = float(document["acceleration_deg_s2"])
        if not np.isfinite([speed, acceleration]).all() or min(speed, acceleration) <= 0:
            raise ValueError("Plan speed and acceleration must be positive and finite")
        delta = np.max(np.abs(np.diff(self.joints, axis=0)), axis=1)
        # Smoothstep peak velocity = 1.5 delta/T, acceleration = 6 delta/T².
        self.durations = np.maximum(.3, np.maximum(1.5 * delta / speed,
                                                   np.sqrt(6 * delta / acceleration)))
        self.arrivals = np.r_[0, np.cumsum(self.durations + .8)]
        self.total = float(self.arrivals[-1] + .8)

    def sample(self, elapsed):
        elapsed = float(np.clip(elapsed, 0, self.total))
        index = min(int(np.searchsorted(self.arrivals, elapsed, side="right") - 1), len(self.joints) - 1)
        if index == len(self.joints) - 1 or elapsed <= self.arrivals[index] + .8:
            return self.joints[index].copy(), index
        fraction = (elapsed - self.arrivals[index] - .8) / self.durations[index]
        smooth = fraction * fraction * (3 - 2 * fraction)
        return self.joints[index] + smooth * (self.joints[index + 1] - self.joints[index]), index + 1


class PlanPlayback:
    def __init__(self, server, document, play=False, start_pose=1):
        self.server = server
        self.timeline = PlanTimeline(document)
        self.ghost = RobotModel()
        self.ghost.add_to_scene(server, "/planned", ghost=True)
        if not 1 <= start_pose <= len(self.timeline.joints):
            raise ValueError("Starting preview pose is outside the saved plan")
        self.elapsed = float(self.timeline.arrivals[start_pose - 1])
        self.last_tick = time.monotonic()
        with server.gui.add_folder("Calibration plan playback"):
            checks = sum(t.get("path_checks", 0) for t in document["targets"])
            server.gui.add_markdown(
                f"**{len(self.timeline.joints)} waypoints · {document.get('capture_pose_count', len(self.timeline.joints))} capture poses · {checks} controller FK checks**\n\n"
                "Blue: planned motion. White: reference pose. Playback never moves the robot. "
                "Timing is approximate. Board visibility and surrounding collisions are not checked.")
            if document.get("purpose") == "wide_base_frame_sweep":
                offsets = document["wide_offsets_mm"]
                server.gui.add_markdown(
                    f"**Wide sweep — ROBOT BASE axes**\n\n"
                    f"Forward +X: {offsets['forward_base_x']:g} mm · "
                    f"Lift +Z: {offsets['lift_base_z']:g} mm · "
                    f"Side Y: ±{offsets['side_base_y']:g} mm. "
                    f"Lower row: Z −{offsets.get('drop_base_z', 0):g} mm, "
                    f"Y ±{offsets.get('lower_side_base_y', offsets['side_base_y']):g} mm. "
                    "These are not calibrated camera directions. Board visibility is unverified.")
            elif document.get("purpose") == "local_bootstrap_sweep" or document.get("radius_mm", 0) <= 50:
                server.gui.add_markdown("**Local bootstrap sweep only.** These poses do not establish "
                                        "coverage of image corners or camera distance. Review camera coverage separately.")
            self.playing = server.gui.add_checkbox("Play", initial_value=play)
            self.rate = server.gui.add_slider("Playback speed", min=.25, max=4., step=.25, initial_value=2.)
            self.slider = server.gui.add_slider("Pose", min=1, max=max(2, len(self.timeline.joints)), step=1, initial_value=1)
            restart = server.gui.add_button("Restart preview")
            self.status = server.gui.add_markdown("")

        @self.slider.on_update
        def select(event):
            if event.client is None:
                return
            self.playing.value = False
            self.elapsed = float(self.timeline.arrivals[min(self.slider.value - 1, len(self.timeline.joints) - 1)])

        @restart.on_click
        def reset(_):
            # Reset the preview instantly: final-to-first is not a checked motion.
            self.elapsed = 0.
            self.last_tick = time.monotonic()
            self.playing.value = True

        @self.playing.on_update
        def toggle(_):
            self.last_tick = time.monotonic()
            if self.playing.value and self.elapsed >= self.timeline.total:
                self.elapsed = 0.

        self.tick(self.last_tick)

    def tick(self, now):
        if self.playing.value:
            self.elapsed = min(self.timeline.total, self.elapsed + max(0, now - self.last_tick) * self.rate.value)
        self.last_tick = now
        if self.elapsed >= self.timeline.total:
            self.playing.value = False
        joints, index = self.timeline.sample(self.elapsed)
        with self.server.atomic():
            self.ghost.set_joints(joints)
            self.ghost.update_scene()
            self.slider.value = index + 1
            self.status.content = (f"Pose **{index + 1}/{len(self.timeline.joints)}** · "
                                   f"{self.elapsed:.1f} / {self.timeline.total:.1f} s\n\n"
                                   + ("Playing" if self.playing.value else "Paused / finished"))
        return {"elapsed_s": self.elapsed, "total_s": self.timeline.total,
                "target_pose": index + 1, "playing": self.playing.value, "joints_deg": joints.tolist()}


def configure_plan(server, path, play=False, start_pose=1):
    if path is None:
        return
    document = read_json(path)
    targets = document["targets"]
    if not targets:
        raise ValueError("Motion plan has no target poses")
    for target in targets:
        finite_array(target["joints_deg"], (7,), "planned joints")
    points = np.asarray([target["T_base_flange"] for target in targets])[:, :3, 3]
    if len(points) > 1:
        server.scene.add_line_segments("/planned_path", points=np.stack((points[:-1], points[1:]), axis=1),
                                       colors=(40, 140, 220), thickness=.003)
    if document.get("purpose") == "wide_base_frame_sweep":
        offsets = document["wide_offsets_mm"]
        server.scene.add_label("/forward_extent", f"BASE +X: {offsets['forward_base_x']:g} mm",
                               position=points[np.argmax(points[:, 0])] + [0, 0, -.05])
        server.scene.add_label("/lift_extent", f"BASE +Z: {offsets['lift_base_z']:g} mm",
                               position=points[np.argmax(points[:, 2])] + [0, 0, .06])
        if offsets.get("drop_base_z", 0) > 0:
            lower = points[np.isclose(points[:, 2], points[:, 2].min(), atol=1e-5)]
            for label, point in (("lower_y_minus", lower[np.argmin(lower[:, 1])]),
                                 ("lower_y_plus", lower[np.argmax(lower[:, 1])])):
                server.scene.add_label("/" + label,
                                       f"LOWER: Y {'−' if label.endswith('minus') else '+'}{offsets['lower_side_base_y']:g} / Z −{offsets['drop_base_z']:g} mm",
                                       position=point + [0, 0, -.05])
    return PlanPlayback(server, document, play, start_pose)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=ROBOT_IP)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--hz", type=float, default=10)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--offline", action="store_true", help="Use saved plan seed; never connect to robot")
    parser.add_argument("--play", action="store_true", help="Automatically play the saved calibration plan")
    parser.add_argument("--start-pose", type=int, default=1, help="Start preview at this waypoint; never sends motion")
    parser.add_argument("--plan", type=Path, help="Optional motion_plan.json for visualization only")
    parser.add_argument("--snapshot", type=Path, default=Path("output/viser/robot_state.json"))
    args = parser.parse_args(argv)
    if args.start_pose < 1 or (args.start_pose != 1 and args.plan is None):
        parser.error("--start-pose requires a plan and a positive waypoint number")
    if args.offline and (args.plan is None or args.demo):
        parser.error("--offline requires --plan and cannot be combined with --demo")
    if args.play and args.plan is None:
        parser.error("--play requires --plan")
    if not np.isfinite(args.hz) or not 0 < args.hz <= 30:
        parser.error("--hz must be between 0 and 30")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")

    import viser

    model = RobotModel()
    arm = server = None
    try:
        if args.offline:
            feedback = dict(read_json(args.plan)["seed"], state="saved", error="not polled", warning="not polled")
        elif args.demo:
            feedback = demo_feedback(model)
        else:
            arm = create_arm(args.ip)
            arm.connect()
            time.sleep(.5)
            if not arm.connected or arm.axis != 7:
                raise RuntimeError("A connected xArm7 is required")
            feedback = read_feedback(arm)
        server = viser.ViserServer(host=args.host, port=args.port, label="xArm7 posture — read only")
        server.gui.configure_theme(control_width="medium", show_share_button=False, brand_color=(40, 125, 180))
        server.scene.set_up_direction("+z")
        server.scene.add_grid("/grid", width=2, height=2, cell_size=.1, section_size=.5)
        server.scene.add_frame("/base", axes_length=.2, axes_radius=.004)
        server.scene.add_label("/base_label", "Robot base", position=(0, 0, -.04))
        server.initial_camera.position = (1.3, 1.3, .95)
        server.initial_camera.look_at = (.2, 0, .35)
        server.initial_camera.up_direction = (0, 0, 1)
        model.add_to_scene(server)
        tcp_axes = server.scene.add_frame("/tcp", axes_length=.12, axes_radius=.003)
        tcp_label = server.scene.add_label("/tcp_label", "SDK TCP", position=(0, 0, 0))
        server.gui.add_markdown("**xArm7 posture viewer**\n\nWhite model: "
                                + ("saved start pose. " if args.offline else "robot joint feedback. ") +
                                "Axes: base / model flange / SDK TCP. All controls are visualization only.")
        live = server.gui.add_checkbox("Live updates", initial_value=True, disabled=args.demo or args.offline)
        status = server.gui.add_markdown("Connecting…")
        with server.gui.add_folder("Joint feedback (degrees)"):
            joint_fields = [server.gui.add_number(f"J{i + 1}", 0.0, disabled=True) for i in range(7)]
        with server.gui.add_folder("SDK TCP (mm / degrees)"):
            tcp_fields = [server.gui.add_number(name, 0.0, disabled=True)
                          for name in ("X", "Y", "Z", "Roll", "Pitch", "Yaw")]
        comparison = server.gui.add_markdown("")
        playback = configure_plan(server, args.plan, args.play, args.start_pose)
        source = "OFFLINE PLAN — no robot connection" if args.offline else "DEMO" if args.demo else args.ip
        print(f"Viewer: http://localhost:{server.get_port()} | {source}", flush=True)
        print(f"Initial joints (deg): {feedback['joints_deg']}", flush=True)
        print(f"Initial TCP (mm, deg): {feedback['tcp_pose_mm_deg']}", flush=True)
        saved_at = 0
        first = True
        while True:
            start = time.monotonic()
            playback_state = playback.tick(start) if playback is not None else None
            if not live.value:
                status.content = "**PAUSED** — frozen view; the real robot may move independently."
                time.sleep(.1)
                continue
            try:
                if not first and arm is not None:
                    feedback = read_feedback(arm)
                model.set_joints(feedback["joints_deg"])
                with server.atomic():
                    model.update_scene()
                    for field, value in zip(joint_fields, feedback["joints_deg"]):
                        field.value = round(value, 3)
                    for field, value in zip(tcp_fields, feedback["tcp_pose_mm_deg"]):
                        field.value = round(value, 3)
                    try:
                        actual_flange = flange_from_tcp(feedback["tcp_pose_mm_deg"],
                                                       feedback["tcp_offset_mm_deg"], feedback["world_offset_mm_deg"])
                        distance, angle = pose_error(model.flange(), actual_flange)
                        tcp = pose_to_transform(feedback["tcp_pose_mm_deg"])
                        tcp_axes.position, tcp_axes.wxyz = tcp[:3, 3], model.quat(tcp[:3, :3])
                        tcp_label.position = tcp[:3, 3] + [0, 0, .04]
                        tcp_axes.visible = tcp_label.visible = True
                        comparison.content = (f"Model flange vs SDK: **{distance:.2f} mm / {angle:.2f}°**\n\n"
                                              + ("Model/frame mismatch: treat the mesh as approximate." if distance > 5 or angle > 3
                                                 else "Model and SDK flange agree within 5 mm / 3°.")
                                              + " During motion, separate SDK reads may add timing error.")
                        feedback["model_flange_error_mm_deg"] = [distance, angle]
                    except ValueError as exc:
                        tcp_axes.visible = tcp_label.visible = False
                        comparison.content = str(exc)
                    status.content = (f"**{source if args.offline or args.demo else 'LIVE — read only'}**\n\n"
                                      f"State: {feedback['state']} · Error: {feedback['error']} · Warning: {feedback['warning']}")
                if first or start - saved_at >= 1:
                    write_json(args.snapshot, dict(feedback, demo=args.demo, offline=args.offline,
                                                   playback=playback_state,
                                                   robot_ip=None if args.demo or args.offline else args.ip))
                    saved_at = start
                if first:
                    print(f"Model / SDK flange error: {feedback.get('model_flange_error_mm_deg', 'unavailable')}", flush=True)
                first = False
            except Exception as exc:
                status.content = f"**STALE — feedback unavailable**\n\n{exc}\n\nLast pose is frozen."
                print(f"Feedback error: {exc}", flush=True)
                # A lost robot link is not replaced by demo or zero joints.
                time.sleep(1)
            time.sleep(max(0, 1 / args.hz - (time.monotonic() - start)))
    finally:
        if arm is not None:
            arm.disconnect()
        if server is not None:
            server.stop()


if __name__ == "__main__":
    run_cli(main)
