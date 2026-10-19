"""English-language Tk desktop teach pendant. Starts disconnected by design."""

import argparse
from datetime import datetime
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from pendant_controller import (
    FEEDBACK_TIMEOUT, HOME_MODE, MODES, PendantWorker, capture_pose, export_poses, velocity_vector,
)
from robot_common import ROBOT_IP


class TeachPendant:
    def __init__(self, root, ip=ROBOT_IP, demo=False):
        self.root, self.demo = root, demo
        self.worker = PendantWorker(demo=demo)
        self.snapshot = {"connected": False, "enabled": False, "generation": 0}
        self.held = None
        self.key_releases = {}
        self.keys_down = set()
        self.closing = False
        self.enabling = False
        self.poses = []
        self.ip = tk.StringVar(value=ip)
        self.mode = tk.StringVar(value="Joint")
        self.axis = tk.IntVar(value=0)
        self.joint_speed = tk.StringVar(value="3")
        self.linear_speed = tk.StringVar(value="5")
        self.angular_speed = tk.StringVar(value="3")
        self.pose_name = tk.StringVar(value="Pose 1")
        self.status = tk.StringVar(value="Disconnected · Connect to inspect the robot")
        self.telemetry = tk.StringVar(value="No robot feedback")
        self.tcp_text = tk.StringVar(value="TCP: —")
        self.offset_text = tk.StringVar(value="Offsets: —")
        self.home_text = tk.StringVar(value="Home J1–J7: not set")
        self.row_labels = [tk.StringVar() for _ in range(7)]
        self.row_values = [tk.StringVar(value="—") for _ in range(7)]
        root.title("xArm7 Teach Pendant" + (" — DEMO" if demo else ""))
        root.geometry(f"1020x{min(920, root.winfo_screenheight() - 90)}")
        root.minsize(950, 650)
        root.configure(bg="#edf1f6")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TFrame", background="#edf1f6")
        style.configure("TLabel", background="#edf1f6", font=("DejaVu Sans", 10))
        style.configure("TButton", font=("DejaVu Sans", 10), padding=7)
        style.configure("TLabelframe", background="#edf1f6")
        style.configure("TLabelframe.Label", background="#edf1f6", font=("DejaVu Sans", 11, "bold"))
        self.build()
        self.update_axis_labels()
        root.bind_all("<ButtonRelease-1>", self.release, add="+")
        # Run pendant shortcuts BEFORE widget class bindings. In particular, Space
        # must not activate a focused Enable button, and arrows must not change a radio selection.
        for sequence, handler in (("<KeyPress-Left>", self.key_press),
                                  ("<KeyPress-Right>", self.key_press),
                                  ("<KeyRelease-Left>", self.key_up),
                                  ("<KeyRelease-Right>", self.key_up),
                                  ("<Escape>", self.escape_stop),
                                  ("<KeyPress-space>", self.space_stop),
                                  ("<KeyRelease-space>", lambda event: None if self.editing() else "break")):
            root.bind_class("PendantKeys", sequence, handler)
        self.install_key_bindings(root)
        root.bind("<FocusOut>", lambda event: root.after_idle(self.check_focus))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.worker.start()
        root.after(40, self.refresh)

    def install_key_bindings(self, widget):
        widget.bindtags(("PendantKeys",) + widget.bindtags())
        for child in widget.winfo_children():
            self.install_key_bindings(child)

    def build(self):
        header = tk.Frame(self.root, bg="#152740", padx=22, pady=16)
        header.pack(fill="x")
        tk.Label(header, text="xArm7  /  Teach Pendant", bg="#152740", fg="white",
                 font=("DejaVu Sans", 20, "bold")).pack(side="left")
        tk.Label(header, text="DEMO · NO NETWORK" if self.demo else "REAL ROBOT",
                 bg="#1e5b68" if self.demo else "#8f3c26", fg="white", padx=12, pady=7,
                 font=("DejaVu Sans", 10, "bold")).pack(side="right")
        tk.Button(header, text="■  STOP  ·  Esc / Space", command=self.stop, bg="#c9323d",
                  activebackground="#ac2631", fg="white", activeforeground="white",
                  relief="flat", padx=12, pady=9, font=("DejaVu Sans", 11, "bold")).pack(side="right", padx=16)
        viewport = ttk.Frame(self.root)
        viewport.pack(fill="both", expand=True)
        canvas = tk.Canvas(viewport, bg="#edf1f6", highlightthickness=0)
        scrollbar = ttk.Scrollbar(viewport, orient="vertical", command=canvas.yview)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        body = ttk.Frame(canvas, padding=18)
        body_window = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(body_window, width=event.width))
        connection = ttk.Frame(body)
        connection.pack(fill="x")
        ttk.Label(connection, text="Robot IP").pack(side="left", padx=(0, 8))
        self.ip_entry = ttk.Entry(connection, textvariable=self.ip, width=19)
        self.ip_entry.pack(side="left")
        self.connect_button = ttk.Button(connection, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=8)
        self.disconnect_button = ttk.Button(connection, text="Disconnect", command=self.disconnect)
        self.disconnect_button.pack(side="left")
        ttk.Button(connection, text="Clear warning 14", command=self.clear_warning).pack(side="right")
        ttk.Label(body, textvariable=self.status, font=("DejaVu Sans", 12, "bold")).pack(anchor="w", pady=(14, 5))
        ttk.Label(body, textvariable=self.telemetry).pack(anchor="w")
        controls = ttk.Frame(body)
        controls.pack(fill="x", pady=(15, 12))
        ttk.Label(controls, text="Jog mode").pack(side="left", padx=(0, 8))
        mode = ttk.Combobox(controls, textvariable=self.mode, values=list(MODES), state="readonly", width=13)
        mode.pack(side="left")
        mode.bind("<<ComboboxSelected>>", self.mode_changed)
        self.enable_button = ttk.Button(controls, text="Enable Jog", command=self.enable)
        self.enable_button.pack(side="left", padx=12)
        middle = ttk.Frame(body)
        middle.pack(fill="x")
        jog = ttk.LabelFrame(middle, text="Hold to jog", padding=12)
        jog.pack(side="left", fill="both", expand=True, padx=(0, 14))
        self.rows = []
        self.jog_buttons = []
        for i in range(7):
            row = ttk.Frame(jog)
            row.pack(fill="x", pady=3)
            ttk.Radiobutton(row, textvariable=self.row_labels[i], variable=self.axis, value=i,
                            command=self.release).pack(side="left")
            ttk.Label(row, textvariable=self.row_values[i], width=14, anchor="e",
                      font=("DejaVu Sans Mono", 12)).pack(side="left", padx=8)
            for direction, label in ((-1, "−"), (1, "+")):
                button = ttk.Button(row, text=label, width=5, takefocus=False)
                button.pack(side="left", padx=4)
                button.bind("<ButtonPress-1>", lambda event, a=i, d=direction: self.mouse_press(a, d))
                button.bind("<Leave>", self.release)
                self.jog_buttons.append(button)
            self.rows.append(row)
        ttk.Label(jog, text="Select an axis, then hold ← / → or a button.").pack(anchor="w", pady=(9, 0))
        settings = ttk.LabelFrame(middle, text="Speed & live TCP", padding=14)
        settings.pack(side="left", fill="both", expand=True)
        for label, variable, units in (("Joint", self.joint_speed, "deg/s · max 10"),
                                      ("TCP linear", self.linear_speed, "mm/s · max 20"),
                                      ("TCP angular", self.angular_speed, "deg/s · max 10")):
            line = ttk.Frame(settings)
            line.pack(fill="x", pady=5)
            ttk.Label(line, text=label, width=12).pack(side="left")
            ttk.Entry(line, textvariable=variable, width=6).pack(side="left", padx=8)
            ttk.Label(line, text=units).pack(side="left")
        ttk.Separator(settings).pack(fill="x", pady=12)
        ttk.Label(settings, textvariable=self.tcp_text, justify="left",
                  font=("DejaVu Sans Mono", 11)).pack(anchor="w")
        ttk.Label(settings, textvariable=self.offset_text, justify="left", wraplength=410).pack(anchor="w", pady=(10, 0))
        ttk.Label(settings, text="Rx / Ry / Rz rotate about frame axes.\nBase XYZ jog keeps orientation; Tool uses TCP axes.",
                  justify="left").pack(anchor="w", pady=(10, 0))
        home = ttk.LabelFrame(body, text="Joint home · nearest of 0° / +45°", padding=10)
        home.pack(fill="x", pady=(14, 0))
        home_bar = ttk.Frame(home)
        home_bar.pack(fill="x")
        self.set_home_button = ttk.Button(home_bar, text="Set Home from Current", command=self.set_home)
        self.set_home_button.pack(side="left")
        self.home_button = ttk.Button(home_bar, text="Hold to Home", takefocus=False)
        self.home_button.pack(side="left", padx=10)
        self.home_button.bind("<ButtonPress-1>", self.begin_home)
        self.home_button.bind("<Leave>", self.release)
        ttk.Label(home_bar, text="Uses Joint speed · Release to stop").pack(side="left")
        ttk.Label(home, textvariable=self.home_text, font=("DejaVu Sans Mono", 11)).pack(anchor="w", pady=(8, 0))
        taught = ttk.LabelFrame(body, text="Taught poses · capture and export", padding=10)
        taught.pack(fill="x", pady=(14, 10))
        bar = ttk.Frame(taught)
        bar.pack(fill="x")
        ttk.Entry(bar, textvariable=self.pose_name, width=24).pack(side="left")
        ttk.Button(bar, text="Capture current pose", command=self.capture).pack(side="left", padx=8)
        ttk.Button(bar, text="Delete selected", command=self.delete_pose).pack(side="left")
        ttk.Button(bar, text="Export JSON…", command=self.export).pack(side="right")
        self.pose_list = tk.Listbox(taught, height=3, relief="flat", font=("DejaVu Sans Mono", 10))
        self.pose_list.pack(fill="x", pady=(8, 0))
        self.log_widget = tk.Text(body, height=3, state="disabled", relief="flat", bg="#dfe6ee",
                                  font=("DejaVu Sans Mono", 9), padx=8, pady=5)
        self.log_widget.pack(fill="both", expand=True)
        note = ("Demo is a UI exercise, not a kinematic or collision simulation."
                if self.demo else "Keep the physical E-stop accessible. Software STOP depends on communication.")
        ttk.Label(body, text=note).pack(anchor="w", pady=(8, 0))

    def log(self, message):
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", f"{datetime.now():%H:%M:%S}  {message}\n")
        if int(self.log_widget.index("end-1c").split(".")[0]) > 150:
            self.log_widget.delete("1.0", "20.0")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def connect(self):
        self.worker.request("connect", self.ip.get().strip())

    def disconnect(self):
        self.release()
        # Disconnect stops only control owned by this pendant, including a pending enable.
        self.worker.stop(force=False)
        self.worker.request("disconnect")

    def enable(self):
        self.release()
        self.enabling = True
        self.worker.request("enable", self.mode.get())

    def stop(self):
        self.release()
        self.enabling = False
        self.worker.stop()

    def mode_changed(self, event=None):
        self.release()
        self.enabling = False
        self.worker.stop(force=False)
        self.axis.set(0)
        self.update_axis_labels()

    def update_axis_labels(self):
        joint = self.mode.get() == "Joint"
        labels = [f"J{i + 1}" for i in range(7)] if joint else ["X", "Y", "Z", "Rx", "Ry", "Rz", ""]
        for variable, label in zip(self.row_labels, labels):
            variable.set(label)
        if joint:
            self.rows[6].pack(fill="x", pady=3, after=self.rows[5])
        else:
            self.rows[6].pack_forget()

    def begin_hold(self, axis, direction, source):
        if self.held is not None:
            return
        if not self.snapshot.get("enabled") or self.snapshot.get("jog_mode") != self.mode.get():
            return
        try:
            variable = self.joint_speed if self.mode.get() == "Joint" else (
                self.linear_speed if axis < 3 else self.angular_speed)
            speed = float(variable.get())
            velocity_vector(self.mode.get(), axis, direction, speed)
        except ValueError as error:
            self.log(str(error))
            return
        self.held = (self.snapshot["generation"], self.mode.get(), axis, direction, speed, source)
        self.worker.hold(*self.held[:5])

    def mouse_press(self, axis, direction):
        self.root.focus_set()
        self.axis.set(axis)
        self.begin_hold(axis, direction, "mouse")
        return "break"

    def set_home(self):
        self.release()
        self.worker.request("set_home")

    def begin_home(self, event=None):
        if self.held is not None or self.snapshot.get("active") or not self.snapshot.get("home_target"):
            return "break"
        try:
            speed = float(self.joint_speed.get())
            velocity_vector("Joint", 0, 1, speed)
        except ValueError as error:
            self.log(str(error))
            return "break"
        self.root.focus_set()
        self.held = (self.snapshot["generation"], HOME_MODE, 0, 1, speed, "home")
        self.worker.hold(*self.held[:5])
        return "break"

    def focus_path(self):
        """Read the Tcl path without resolving Tk-owned popups as Python widgets."""
        try:
            return str(self.root.tk.call("focus", "-displayof", self.root._w))
        except tk.TclError:
            return ""

    def has_control_focus(self):
        path = self.focus_path()
        if not path or path == "none":
            return False
        try:
            # A combobox popdown is a separate Tk toplevel. Treat it like a dialog:
            # it must not keep a held motion alive while it consumes input.
            return str(self.root.tk.call("winfo", "toplevel", path)) == self.root._w
        except tk.TclError:
            return False

    def editing(self):
        path = self.focus_path()
        if not path or path == "none":
            return True
        try:
            return str(self.root.tk.call("winfo", "class", path)) in (
                "Entry", "TEntry", "TCombobox", "Text", "Listbox", "Spinbox", "TSpinbox",
            )
        except tk.TclError:
            return True

    def key_press(self, event):
        pending = self.key_releases.pop(event.keysym, None)
        if pending is not None:
            self.root.after_cancel(pending)
        if event.keysym in self.keys_down:
            return "break" if not self.editing() else None
        self.keys_down.add(event.keysym)
        if self.editing():
            return
        if self.held is not None:
            return "break"
        self.begin_hold(self.axis.get(), -1 if event.keysym == "Left" else 1, event.keysym)
        return "break"

    def key_up(self, event):
        # X11 key-repeat emits a release immediately followed by another press.
        pending = self.key_releases.pop(event.keysym, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.key_releases[event.keysym] = self.root.after_idle(lambda: self.finish_key_up(event.keysym))
        return None if self.editing() else "break"

    def finish_key_up(self, key):
        self.key_releases.pop(key, None)
        self.keys_down.discard(key)
        if self.held and self.held[-1] == key:
            self.release()

    def release(self, event=None):
        self.held = None
        self.worker.release()

    def space_stop(self, event):
        self.stop()
        return None if self.editing() else "break"

    def escape_stop(self, event):
        self.stop()
        return "break"

    def check_focus(self):
        if not self.closing and (not self.has_control_focus() or (self.held and self.editing())):
            self.release()
            if self.snapshot.get("enabled") or self.enabling:
                self.stop()

    def clear_warning(self):
        self.release()
        self.worker.request("clear_warning_14")

    def capture(self):
        try:
            pose = capture_pose(self.snapshot, self.pose_name.get(), self.ip.get(), self.demo)
            self.poses.append(pose)
            xyz = ", ".join(f"{v:.2f}" for v in pose["tcp_mm_deg"][:3])
            self.pose_list.insert("end", f"{pose['name']}   XYZ [{xyz}] mm")
            self.pose_name.set(f"Pose {len(self.poses) + 1}")
            self.log(f"Captured {pose['name']}. Export JSON to save to disk.")
        except ValueError as error:
            self.log(str(error))

    def delete_pose(self):
        for i in reversed(self.pose_list.curselection()):
            del self.poses[i]
            self.pose_list.delete(i)

    def export(self):
        self.release()
        self.worker.stop(force=False)
        if not self.poses:
            self.log("Capture a pose before exporting.")
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json",
                                          initialfile="taught_poses.json",
                                          filetypes=[("JSON poses", "*.json")])
        if path:
            try:
                export_poses(path, self.poses)
                self.log(f"Saved {len(self.poses)} pose(s) to {path}")
            except (OSError, ValueError) as error:
                messagebox.showerror("Export failed", str(error), parent=self.root)

    def refresh(self):
        if self.closing:
            return
        self.snapshot, messages = self.worker.view()
        for message in messages:
            self.log(message)
        if messages:
            self.enabling = False
        s = self.snapshot
        age = time.monotonic() - s.get("received", 0)
        fresh = s["connected"] and age <= FEEDBACK_TIMEOUT
        if self.held:
            preparing_home = self.held[1] == HOME_MODE and s.get("home_target") is not None
            if ((not s["enabled"] and not preparing_home) or not fresh or self.held[0] != s["generation"]
                    or not self.has_control_focus()):
                self.release()
            else:
                self.worker.hold(*self.held[:5])
        self.connect_button.configure(state="disabled" if s.get("has_session") else "normal")
        self.ip_entry.configure(state="disabled" if s.get("has_session") else "normal")
        self.disconnect_button.configure(state="normal" if s.get("has_session") else "disabled")
        self.enable_button.configure(state="normal" if fresh and not s["enabled"] and not self.enabling and not self.held else "disabled")
        self.set_home_button.configure(state="normal" if fresh and not s.get("active") and not self.held else "disabled")
        self.home_button.configure(state="normal" if fresh and s.get("home_target") is not None
                                   and (not s.get("active") or s.get("jog_mode") == HOME_MODE) else "disabled")
        target = s.get("home_target")
        self.home_text.set("Home J1–J7 (deg): [" + ", ".join(f"{v:g}" for v in target) + "]"
                           if target is not None else "Home J1–J7: not set")
        for button in self.jog_buttons:
            button.configure(state="normal" if fresh and s["enabled"] and s.get("jog_mode") != HOME_MODE else "disabled")
        self.status.set("HOMING · Release to stop" if s.get("active") and s.get("jog_mode") == HOME_MODE else
                        "JOGGING · Release to stop" if s.get("active") else
                        "ENABLED · Hold a jog control to move" if s["enabled"] and fresh else
                        "CONNECTED · Jogging disabled" if fresh else
                        "STALE FEEDBACK · Jogging unavailable" if s["connected"] else "DISCONNECTED")
        if s.get("tcp"):
            self.telemetry.set(f"Firmware {s['firmware']}   |   Mode {s['robot_mode']}   State {s['state']}"
                               f"   |   Error {s['error']}   Warning {s['warning']}   |   Feedback {age:.2f} s ago")
            values = s["joints"] if self.mode.get() == "Joint" else s["tcp"]
            for i, value in enumerate(values):
                unit = "mm" if self.mode.get() != "Joint" and i < 3 else "deg"
                # Rx/Ry/Rz jog frame axes, not Euler-angle components. Show RPY only in the TCP panel.
                self.row_values[i].set("—" if self.mode.get() != "Joint" and i >= 3 else f"{value:8.2f} {unit}")
            x, y, z, roll, pitch, yaw = s["tcp"]
            self.tcp_text.set(f"X {x:8.2f}  Y {y:8.2f}  Z {z:8.2f} mm\n"
                              f"R {roll:8.2f}  P {pitch:8.2f}  Y {yaw:8.2f} deg")
            self.offset_text.set(f"TCP offset: {s['tcp_offset']}\nWorld offset: {s['world_offset']}")
        else:
            self.telemetry.set("No robot feedback")
            self.tcp_text.set("TCP: —")
            self.offset_text.set("Offsets: —")
            for value in self.row_values:
                value.set("—")
        self.root.after(40, self.refresh)

    def close(self):
        self.closing = True
        self.release()
        self.worker.stop(close=True)
        self.status.set("Closing · waiting for robot worker to disconnect")
        self.await_close()

    def await_close(self):
        if self.worker.is_alive():
            self.root.after(50, self.await_close)
        else:
            _, messages = self.worker.view()
            for message in messages:
                print(message, file=sys.stderr)
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=ROBOT_IP, help="Robot IP (default: %(default)s)")
    parser.add_argument("--demo", action="store_true", help="Use a local UI demo; never connect to hardware")
    args = parser.parse_args()
    try:
        root = tk.Tk()
    except tk.TclError as error:
        parser.exit(1, f"A graphical desktop is required: {error}\nRun this command in a desktop terminal.\n")
    app = TeachPendant(root, args.ip, args.demo)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()
        root.mainloop()


if __name__ == "__main__":
    main()
