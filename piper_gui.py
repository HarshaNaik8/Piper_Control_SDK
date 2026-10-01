"""Piper Robot Arm Controller - Advanced Graphical User Interface.

Features:
  1. Full Power & Motor Control:
     - Enable All Motors (Stiff / Active Holding Torque)
     - Disable All Motors (Unpowered / Dead Weight)
     - Emergency Stop
  2. Gripper Freedom:
     - Make Gripper Loose (for easy hand-guidance during demonstration)
     - Make Gripper Stiff / Position Hold
     - Quick Open (70mm) / Close (0mm)
  3. Software Trajectory Recorder & Mimic Player (The Dataset Collection Engine):
     - Record J1-J6 + Gripper at 50 Hz while moving the arm by hand
     - Smoothly replay recorded trajectory with adjustable speed (0.5x - 2.0x)
     - Loop replay across multiple episodes automatically
  4. Firmware Trajectory Trigger:
     - Software substitute for double-tapping the physical button
  5. Real-Time Telemetry & Event Logging
"""

import os
import sys
import time
import glob
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from typing import Optional, List

from piper_controller import PiperRobotController
from config import ControlMode, ArmStatus


class PiperGUI(tk.Tk):
    """Modern Tkinter GUI for the AgileX Piper 6-DOF Robot Arm."""

    def __init__(self):
        super().__init__()
        self.title("Piper Arm Controller - AgileX CAN Interface Suite")
        self.geometry("860x820")
        self.minsize(800, 750)

        self._setup_styles()

        self.controller: Optional[PiperRobotController] = None
        self._polling_active = False
        self._latest_trajectory_file: Optional[str] = None

        self._build_ui()

    def _setup_styles(self):
        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.style.configure("TFrame", background="#F4F6F9")
        self.style.configure("Header.TLabel", font=("Segoe UI", 13, "bold"), background="#F4F6F9", foreground="#1A252F")
        self.style.configure("Section.TLabelframe", background="#F4F6F9")
        self.style.configure("Section.TLabelframe.Label", font=("Segoe UI", 10, "bold"), foreground="#2C3E50")
        self.configure(bg="#F4F6F9")

    def _build_ui(self):
        # 1. Top Header
        header = ttk.Frame(self)
        header.pack(fill="x", padx=16, pady=(10, 4))

        title_lbl = ttk.Label(header, text="AgileX Piper Robot Arm Controller", style="Header.TLabel")
        title_lbl.pack(side="left")

        self.sim_var = tk.BooleanVar(value=False)
        sim_chk = ttk.Checkbutton(header, text="Simulation (Offline) Mode", variable=self.sim_var)
        sim_chk.pack(side="right")

        # 2. CAN Connection Settings
        conn_frame = ttk.LabelFrame(self, text="CAN Connection", style="Section.TLabelframe")
        conn_frame.pack(fill="x", padx=16, pady=4)

        conn_box = ttk.Frame(conn_frame)
        conn_box.pack(fill="x", padx=10, pady=6)

        ttk.Label(conn_box, text="Interface:").grid(row=0, column=0, sticky="w", padx=4)
        self.iface_combo = ttk.Combobox(conn_box, values=["agx_cando", "slcan", "socketcan", "virtual"], width=12)
        self.iface_combo.set("agx_cando")
        self.iface_combo.grid(row=0, column=1, padx=4)

        ttk.Label(conn_box, text="Channel:").grid(row=0, column=2, sticky="w", padx=(12, 4))
        self.channel_entry = ttk.Entry(conn_box, width=6)
        self.channel_entry.insert(0, "0")
        self.channel_entry.grid(row=0, column=3, padx=4)

        self.btn_connect = tk.Button(
            conn_box, text="Connect", bg="#27AE60", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=14, pady=2,
            command=self._toggle_connection
        )
        self.btn_connect.grid(row=0, column=4, padx=(14, 6))

        self.lbl_conn_status = tk.Label(
            conn_box, text="● Disconnected", fg="#E74C3C",
            bg="#F4F6F9", font=("Segoe UI", 10, "bold")
        )
        self.lbl_conn_status.grid(row=0, column=5, padx=8)

        # 3. Live Robot Status & Telemetry
        status_frame = ttk.LabelFrame(self, text="Live Robot Telemetry", style="Section.TLabelframe")
        status_frame.pack(fill="x", padx=16, pady=4)

        stat_box = ttk.Frame(status_frame)
        stat_box.pack(fill="x", padx=10, pady=6)

        ttk.Label(stat_box, text="Control Mode:", font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w")
        self.lbl_mode = ttk.Label(stat_box, text="--", foreground="#2980B9", font=("Segoe UI", 9))
        self.lbl_mode.grid(row=0, column=1, sticky="w", padx=(4, 16))

        ttk.Label(stat_box, text="Arm Status:", font=("Segoe UI", 9, "bold")).grid(row=0, column=2, sticky="w")
        self.lbl_state = ttk.Label(stat_box, text="--", foreground="#27AE60", font=("Segoe UI", 9, "bold"))
        self.lbl_state.grid(row=0, column=3, sticky="w", padx=(4, 16))

        ttk.Label(stat_box, text="CAN FPS:", font=("Segoe UI", 9, "bold")).grid(row=0, column=4, sticky="w")
        self.lbl_fps = ttk.Label(stat_box, text="0.0 Hz", font=("Segoe UI", 9))
        self.lbl_fps.grid(row=0, column=5, sticky="w", padx=4)

        # Joint angle cards
        joint_box = ttk.Frame(stat_box)
        joint_box.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(6, 2))

        self.joint_labels = []
        for i in range(6):
            lbl = ttk.Label(joint_box, text=f"J{i+1}: 0.0°", width=11, relief="groove", anchor="center")
            lbl.grid(row=0, column=i, padx=3, pady=2)
            self.joint_labels.append(lbl)

        self.lbl_gripper = ttk.Label(stat_box, text="Gripper Stroke: 0.0 mm", font=("Segoe UI", 9, "bold"), foreground="#8E44AD")
        self.lbl_gripper.grid(row=2, column=0, columnspan=6, sticky="w", pady=(4, 0))

        # 4. Robot Power & Gripper Controls
        pwr_frame = ttk.LabelFrame(self, text="Robot Power & Gripper Freedom", style="Section.TLabelframe")
        pwr_frame.pack(fill="x", padx=16, pady=4)

        pwr_box = ttk.Frame(pwr_frame)
        pwr_box.pack(fill="x", padx=10, pady=6)

        # Left: Arm motors
        self.btn_enable = tk.Button(
            pwr_box, text="⚡ Enable All Motors (Stiff / Hold)", bg="#16A085", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=10, pady=3,
            state="disabled", command=self._cmd_enable_arm
        )
        self.btn_enable.grid(row=0, column=0, padx=3, pady=2)

        self.btn_disable = tk.Button(
            pwr_box, text="🔓 Disable All Motors (Dead Weight)", bg="#95A5A6", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=10, pady=3,
            state="disabled", command=self._cmd_disable_arm
        )
        self.btn_disable.grid(row=0, column=1, padx=3, pady=2)

        # Right: Gripper controls (Solving stiff gripper!)
        self.btn_grp_loose = tk.Button(
            pwr_box, text="🔓 Make Gripper Loose (Drag-Teach)", bg="#E67E22", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=10, pady=3,
            state="disabled", command=self._cmd_gripper_loose
        )
        self.btn_grp_loose.grid(row=0, column=2, padx=(12, 3), pady=2)

        self.btn_grp_stiff = tk.Button(
            pwr_box, text="🔒 Gripper Stiff / Hold", bg="#7F8C8D", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=8, pady=3,
            state="disabled", command=self._cmd_gripper_stiff
        )
        self.btn_grp_stiff.grid(row=0, column=3, padx=3, pady=2)

        self.btn_grp_open = tk.Button(
            pwr_box, text="Open (70mm)", bg="#BDC3C7", fg="#2C3E50",
            font=("Segoe UI", 8), relief="flat", padx=6, pady=2,
            state="disabled", command=lambda: self._cmd_set_gripper(70.0)
        )
        self.btn_grp_open.grid(row=0, column=4, padx=2, pady=2)

        self.btn_grp_close = tk.Button(
            pwr_box, text="Close (0mm)", bg="#BDC3C7", fg="#2C3E50",
            font=("Segoe UI", 8), relief="flat", padx=6, pady=2,
            state="disabled", command=lambda: self._cmd_set_gripper(0.0)
        )
        self.btn_grp_close.grid(row=0, column=5, padx=2, pady=2)

        self.btn_estop = tk.Button(
            pwr_box, text="🛑 EMERGENCY STOP", bg="#D63031", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=3,
            state="disabled", command=self._cmd_estop
        )
        self.btn_estop.grid(row=0, column=6, padx=(14, 2), pady=2)

        # 5. Software Trajectory Recorder & Mimic Player (The Dataset Collection Engine)
        soft_frame = ttk.LabelFrame(self, text="Trajectory Recording & Mimic Playback (Dataset Engine)", style="Section.TLabelframe")
        soft_frame.pack(fill="x", padx=16, pady=4)

        soft_box = ttk.Frame(soft_frame)
        soft_box.pack(fill="x", padx=10, pady=6)

        # Recording row
        self.btn_rec_start = tk.Button(
            soft_box, text="🔴 Start Software Recording", bg="#8E44AD", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_start_software_record
        )
        self.btn_rec_start.grid(row=0, column=0, padx=4, pady=3)

        self.btn_rec_stop = tk.Button(
            soft_box, text="⏹ Stop & Save Trajectory", bg="#7F8C8D", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_stop_software_record
        )
        self.btn_rec_stop.grid(row=0, column=1, padx=4, pady=3)

        self.lbl_record_info = ttk.Label(soft_box, text="Ready to record.", font=("Segoe UI", 9, "italic"), foreground="#7F8C8D")
        self.lbl_record_info.grid(row=0, column=2, columnspan=3, sticky="w", padx=8)

        # Replay row
        sep1 = ttk.Separator(soft_box, orient="horizontal")
        sep1.grid(row=1, column=0, columnspan=6, sticky="ew", pady=6)

        self.btn_mimic_play = tk.Button(
            soft_box, text="▶  Mimic / Replay Trajectory (Software)", bg="#2980B9", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=14, pady=5,
            state="disabled", command=self._cmd_replay_software
        )
        self.btn_mimic_play.grid(row=2, column=0, padx=4, pady=3)

        self.btn_soft_pause = tk.Button(
            soft_box, text="⏸ Pause", bg="#F39C12", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=10, pady=4,
            state="disabled", command=self._cmd_pause_replay
        )
        self.btn_soft_pause.grid(row=2, column=1, padx=4, pady=3)

        self.btn_soft_resume = tk.Button(
            soft_box, text="⏯ Resume", bg="#27AE60", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=10, pady=4,
            state="disabled", command=self._cmd_resume_replay
        )
        self.btn_soft_resume.grid(row=2, column=2, padx=4, pady=3)

        self.btn_soft_stop = tk.Button(
            soft_box, text="⏹ Stop Replay", bg="#C0392B", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=10, pady=4,
            state="disabled", command=self._cmd_stop_replay
        )
        self.btn_soft_stop.grid(row=2, column=3, padx=4, pady=3)

        # Replay settings (Speed & Loop)
        opt_box = ttk.Frame(soft_box)
        opt_box.grid(row=2, column=4, columnspan=2, sticky="e", padx=6)

        ttk.Label(opt_box, text="Speed:").pack(side="left", padx=2)
        self.speed_combo = ttk.Combobox(opt_box, values=["0.5x", "1.0x", "1.5x", "2.0x"], width=5)
        self.speed_combo.set("1.0x")
        self.speed_combo.pack(side="left", padx=2)

        ttk.Label(opt_box, text="Loops:").pack(side="left", padx=(8, 2))
        self.loop_spin = ttk.Spinbox(opt_box, from_=1, to=100, width=4)
        self.loop_spin.set(1)
        self.loop_spin.pack(side="left", padx=2)

        # 6. Firmware Trajectory Playback (Physical Button Double-Tap Trigger)
        hw_frame = ttk.LabelFrame(self, text="Firmware Trajectory Playback (Physical Button Trigger)", style="Section.TLabelframe")
        hw_frame.pack(fill="x", padx=16, pady=4)

        hw_box = ttk.Frame(hw_frame)
        hw_box.pack(fill="x", padx=10, pady=6)

        ttk.Label(
            hw_box,
            text="Note: Use this to replay a motion previously recorded via the PHYSICAL green button.",
            font=("Segoe UI", 8, "italic"), foreground="#5D6D7E"
        ).grid(row=0, column=0, columnspan=5, sticky="w", pady=(0, 4))

        self.btn_hw_play = tk.Button(
            hw_box, text="▶ Trigger Firmware Playback (0x150, 0x03)", bg="#16A085", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_execute_hw_trajectory
        )
        self.btn_hw_play.grid(row=1, column=0, padx=4, pady=2)

        self.btn_hw_pause = tk.Button(
            hw_box, text="⏸ Pause", bg="#F39C12", fg="white",
            font=("Segoe UI", 8), relief="flat", padx=8, pady=3,
            state="disabled", command=self._cmd_pause_hw
        )
        self.btn_hw_pause.grid(row=1, column=1, padx=4, pady=2)

        self.btn_hw_resume = tk.Button(
            hw_box, text="⏯ Resume", bg="#27AE60", fg="white",
            font=("Segoe UI", 8), relief="flat", padx=8, pady=3,
            state="disabled", command=self._cmd_resume_hw
        )
        self.btn_hw_resume.grid(row=1, column=2, padx=4, pady=2)

        self.btn_hw_stop = tk.Button(
            hw_box, text="⏹ Stop", bg="#C0392B", fg="white",
            font=("Segoe UI", 8), relief="flat", padx=8, pady=3,
            state="disabled", command=self._cmd_stop_hw
        )
        self.btn_hw_stop.grid(row=1, column=3, padx=4, pady=2)

        self.btn_hw_start = tk.Button(
            hw_box, text="📍 Move to Start", bg="#34495E", fg="white",
            font=("Segoe UI", 8), relief="flat", padx=8, pady=3,
            state="disabled", command=self._cmd_start_pt_hw
        )
        self.btn_hw_start.grid(row=1, column=4, padx=4, pady=2)

        # 7. Console Activity Log
        log_frame = ttk.LabelFrame(self, text="Activity & Event Log", style="Section.TLabelframe")
        log_frame.pack(fill="both", expand=True, padx=16, pady=(4, 10))

        self.log_text = tk.Text(log_frame, height=6, bg="#1E1E1E", fg="#D4D4D4", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, padx=6, pady=4)

        self._log("System initialized. Connect CAN adapter to begin.")

    def _log(self, text: str):
        now_str = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{now_str}] {text}\n")
        self.log_text.see(tk.END)

    def _toggle_connection(self):
        if self.controller and self.controller.is_connected():
            self._polling_active = False
            self.controller.disconnect()
            self.controller = None
            self.btn_connect.config(text="Connect", bg="#27AE60")
            self.lbl_conn_status.config(text="● Disconnected", fg="#E74C3C")
            self._set_controls_state("disabled")
            self._log("Disconnected from Piper arm.")
        else:
            iface = self.iface_combo.get().strip()
            channel = self.channel_entry.get().strip()
            mock = self.sim_var.get()

            self._log(f"Connecting to {iface}:{channel} (mock={mock})...")
            self.controller = PiperRobotController(
                interface=iface,
                channel=channel,
                mock=mock,
            )
            success = self.controller.connect(timeout=2.5)
            if success:
                self.btn_connect.config(text="Disconnect", bg="#E67E22")
                self.lbl_conn_status.config(text="● Connected", fg="#27AE60")
                self._set_controls_state("normal")
                self._log("Connected to Piper arm! Telemetry stream active.")
                self._start_polling()
            else:
                self.controller = None
                self._log("Connection failed. Check USB-CAN adapter, cables, and 24V power.")
                messagebox.showerror("Connection Failed", "Could not connect to Piper. Check hardware or tick Simulation Mode.")

    def _set_controls_state(self, state: str):
        buttons = [
            self.btn_enable, self.btn_disable, self.btn_grp_loose, self.btn_grp_stiff,
            self.btn_grp_open, self.btn_grp_close, self.btn_estop,
            self.btn_rec_start, self.btn_rec_stop, self.btn_mimic_play,
            self.btn_soft_pause, self.btn_soft_resume, self.btn_soft_stop,
            self.btn_hw_play, self.btn_hw_pause, self.btn_hw_resume, self.btn_hw_stop, self.btn_hw_start
        ]
        for b in buttons:
            b.config(state=state)

    def _start_polling(self):
        self._polling_active = True

        def _poll():
            while self._polling_active and self.controller and self.controller.is_connected():
                try:
                    st = self.controller.get_status()
                    joints = self.controller.get_joint_angles()
                    grp = self.controller.get_gripper_stroke()
                    rec_info = self.controller.get_recorded_trajectory_info()

                    self.after(0, self._update_ui, st, joints, grp, rec_info)
                except Exception:
                    pass
                time.sleep(0.1)

        threading.Thread(target=_poll, daemon=True).start()

    def _update_ui(self, st: dict, joints: list, grp: float, rec_info: dict):
        self.lbl_mode.config(text=st["ctrl_mode_name"])
        self.lbl_state.config(text=st["arm_status_name"])
        self.lbl_fps.config(text=f"{st['fps']:.1f} Hz")
        self.lbl_gripper.config(text=f"Gripper Stroke: {grp:.1f} mm")

        # Visual color badge
        if st["is_executing"]:
            self.lbl_state.config(foreground="#E67E22")
        elif st["is_recording"]:
            self.lbl_state.config(foreground="#8E44AD")
        else:
            self.lbl_state.config(foreground="#27AE60")

        for i, val in enumerate(joints[:6]):
            self.joint_labels[i].config(text=f"J{i+1}: {val:6.1f}°")

        if rec_info["is_recording"]:
            self.lbl_record_info.config(
                text=f"Recording... {rec_info['num_points']} waypoints ({rec_info['duration_sec']:.1f}s)",
                foreground="#C0392B"
            )

    # -------------------------------------------------------------------------
    # Command Callbacks
    # -------------------------------------------------------------------------

    def _cmd_enable_arm(self):
        self._log("Command: WAKE UP & ENABLE MOTORS (Joints & Gripper stiff / position hold)")
        if self.controller:
            self.controller.enable_arm()

    def _cmd_disable_arm(self):
        self._log("Command: DISABLE ALL MOTORS (Unpowered / Dead Weight)")
        if self.controller:
            self.controller.disable_arm()

    def _cmd_gripper_loose(self):
        self._log("Command: MAKE GRIPPER LOOSE (Motor 7 power cut for easy hand demonstration)")
        if self.controller:
            self.controller.make_gripper_loose()

    def _cmd_gripper_stiff(self):
        self._log("Command: MAKE GRIPPER STIFF (Motor 7 holding torque applied)")
        if self.controller:
            self.controller.make_gripper_stiff()

    def _cmd_set_gripper(self, stroke_mm: float):
        self._log(f"Command: SET GRIPPER STROKE to {stroke_mm} mm")
        if self.controller:
            self.controller.set_gripper_stroke(stroke_mm)

    def _cmd_estop(self):
        self._log("Command: !!! EMERGENCY STOP !!!")
        if self.controller:
            self.controller.emergency_stop()

    # Software Trajectory
    def _cmd_start_software_record(self):
        self._log(">>> STARTED SOFTWARE TRAJECTORY RECORDING (Move robot arm and gripper by hand now!)")
        if self.controller:
            self.controller.start_software_recording(sample_hz=50.0)

    def _cmd_stop_software_record(self):
        if self.controller:
            save_path = self.controller.stop_software_recording()
            if save_path:
                self._latest_trajectory_file = save_path
                self.lbl_record_info.config(
                    text=f"Saved: {os.path.basename(save_path)}",
                    foreground="#27AE60"
                )
                self._log(f">>> RECORDING STOPPED. Saved to: {save_path}")

    def _cmd_replay_software(self):
        if not self._latest_trajectory_file:
            # Check if any trajectory files exist in trajectories/
            saved = glob.glob("trajectories/*.json")
            if saved:
                saved.sort(key=os.path.getmtime, reverse=True)
                self._latest_trajectory_file = saved[0]
            else:
                messagebox.showwarning("No Trajectory", "Please record a software trajectory first (or place one in trajectories/).")
                return

        # Parse speed and loops
        speed_text = self.speed_combo.get().replace("x", "")
        speed_factor = float(speed_text) if speed_text else 1.0
        try:
            loop_count = int(self.loop_spin.get())
        except ValueError:
            loop_count = 1

        self._log(f">>> REPLAYING TRAJECTORY '{os.path.basename(self._latest_trajectory_file)}' (Speed: {speed_factor}x, Loops: {loop_count})...")
        if self.controller:
            self.controller.replay_software_trajectory(
                trajectory=self._latest_trajectory_file,
                speed_factor=speed_factor,
                loop_count=loop_count,
            )

    def _cmd_pause_replay(self):
        self._log("Command: PAUSE SOFTWARE REPLAY")
        if self.controller:
            self.controller.pause_software_replay()

    def _cmd_resume_replay(self):
        self._log("Command: RESUME SOFTWARE REPLAY")
        if self.controller:
            self.controller.resume_software_replay()

    def _cmd_stop_replay(self):
        self._log("Command: STOP SOFTWARE REPLAY")
        if self.controller:
            self.controller.stop_software_replay()

    # Firmware Trajectory
    def _cmd_execute_hw_trajectory(self):
        self._log("Command: TRIGGER FIRMWARE PLAYBACK (CAN 0x150, 0x03)")
        if self.controller:
            self.controller.execute_firmware_trajectory()

    def _cmd_pause_hw(self):
        self._log("Command: PAUSE FIRMWARE PLAYBACK")
        if self.controller:
            self.controller.pause_firmware_trajectory()

    def _cmd_resume_hw(self):
        self._log("Command: RESUME FIRMWARE PLAYBACK")
        if self.controller:
            self.controller.resume_firmware_trajectory()

    def _cmd_stop_hw(self):
        self._log("Command: TERMINATE FIRMWARE PLAYBACK")
        if self.controller:
            self.controller.stop_firmware_trajectory()

    def _cmd_start_pt_hw(self):
        self._log("Command: MOVE TO FIRMWARE TRAJECTORY START")
        if self.controller:
            self.controller.move_to_trajectory_start()


def main():
    app = PiperGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
