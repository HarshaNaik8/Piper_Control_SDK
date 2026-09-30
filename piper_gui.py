"""Piper Robot Arm Laptop Controller - Graphical User Interface (Level 7).

Implements the user interface specified in the Piper Control Planning document.
Allows connecting to the Piper arm over CAN/USB, viewing live telemetric feedback,
and executing the taught trajectory without physical button presses.
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from typing import Optional

from piper_controller import PiperRobotController
from config import ControlMode, ArmStatus


class PiperGUI(tk.Tk):
    """Tkinter-based GUI for controlling AgileX Piper robotic arm."""

    def __init__(self):
        super().__init__()
        self.title("Piper Arm Controller - AgileX CAN Interface")
        self.geometry("780x720")
        self.minsize(700, 650)

        # Style configuration
        self._setup_styles()

        self.controller: Optional[PiperRobotController] = None
        self._polling_active = False

        self._build_ui()

    def _setup_styles(self):
        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.style.configure("TFrame", background="#F4F6F9")
        self.style.configure("Header.TLabel", font=("Segoe UI", 14, "bold"), background="#F4F6F9", foreground="#2C3E50")
        self.style.configure("Section.TLabelframe", background="#F4F6F9")
        self.style.configure("Section.TLabelframe.Label", font=("Segoe UI", 10, "bold"), foreground="#34495E")
        self.configure(bg="#F4F6F9")

    def _build_ui(self):
        # Top Header
        header_frame = ttk.Frame(self)
        header_frame.pack(fill="x", padx=16, pady=(12, 6))

        title_lbl = ttk.Label(header_frame, text="AgileX Piper Robot Arm Controller", style="Header.TLabel")
        title_lbl.pack(side="left")

        self.sim_var = tk.BooleanVar(value=False)
        sim_chk = ttk.Checkbutton(header_frame, text="Simulation (Offline) Mode", variable=self.sim_var)
        sim_chk.pack(side="right")

        # Connection Frame
        conn_frame = ttk.LabelFrame(self, text="CAN Connection Settings", style="Section.TLabelframe")
        conn_frame.pack(fill="x", padx=16, pady=6)

        inner_conn = ttk.Frame(conn_frame)
        inner_conn.pack(fill="x", padx=10, pady=8)

        ttk.Label(inner_conn, text="Interface:").grid(row=0, column=0, sticky="w", padx=4)
        self.iface_combo = ttk.Combobox(inner_conn, values=["agx_cando", "slcan", "socketcan", "virtual"], width=12)
        self.iface_combo.set("agx_cando")
        self.iface_combo.grid(row=0, column=1, padx=4)

        ttk.Label(inner_conn, text="Channel:").grid(row=0, column=2, sticky="w", padx=(12, 4))
        self.channel_entry = ttk.Entry(inner_conn, width=8)
        self.channel_entry.insert(0, "0")
        self.channel_entry.grid(row=0, column=3, padx=4)

        self.btn_connect = tk.Button(
            inner_conn, text="Connect", bg="#27AE60", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=14, pady=3,
            command=self._toggle_connection
        )
        self.btn_connect.grid(row=0, column=4, padx=(16, 6))

        self.lbl_conn_status = tk.Label(
            inner_conn, text="● Disconnected", fg="#E74C3C",
            bg="#F4F6F9", font=("Segoe UI", 10, "bold")
        )
        self.lbl_conn_status.grid(row=0, column=5, padx=8)

        # Status & Telemetry Frame
        status_frame = ttk.LabelFrame(self, text="Robot Status & Telemetry", style="Section.TLabelframe")
        status_frame.pack(fill="x", padx=16, pady=6)

        inner_status = ttk.Frame(status_frame)
        inner_status.pack(fill="x", padx=10, pady=8)

        # Labels for telemetry
        ttk.Label(inner_status, text="Control Mode:", font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w")
        self.lbl_mode = ttk.Label(inner_status, text="--", foreground="#2980B9", font=("Segoe UI", 9))
        self.lbl_mode.grid(row=0, column=1, sticky="w", padx=(4, 20))

        ttk.Label(inner_status, text="Arm Status:", font=("Segoe UI", 9, "bold")).grid(row=0, column=2, sticky="w")
        self.lbl_state = ttk.Label(inner_status, text="--", foreground="#27AE60", font=("Segoe UI", 9, "bold"))
        self.lbl_state.grid(row=0, column=3, sticky="w", padx=(4, 20))

        ttk.Label(inner_status, text="CAN FPS:", font=("Segoe UI", 9, "bold")).grid(row=0, column=4, sticky="w")
        self.lbl_fps = ttk.Label(inner_status, text="0.0 Hz", font=("Segoe UI", 9))
        self.lbl_fps.grid(row=0, column=5, sticky="w", padx=4)

        # Joint Angles Display
        joint_box = ttk.Frame(inner_status)
        joint_box.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(8, 0))

        self.joint_labels = []
        for i in range(6):
            lbl = ttk.Label(joint_box, text=f"J{i+1}: 0.0°", width=12, relief="groove", anchor="center")
            lbl.grid(row=0, column=i, padx=3, pady=2)
            self.joint_labels.append(lbl)

        self.lbl_gripper = ttk.Label(inner_status, text="Gripper Stroke: 0.0 mm", font=("Segoe UI", 9))
        self.lbl_gripper.grid(row=2, column=0, columnspan=6, sticky="w", pady=(6, 0))

        # Trajectory Control Frame (Primary Goal)
        action_frame = ttk.LabelFrame(self, text="Trajectory Playback Controls (Double-Tap Replacement)", style="Section.TLabelframe")
        action_frame.pack(fill="x", padx=16, pady=6)

        inner_actions = ttk.Frame(action_frame)
        inner_actions.pack(fill="x", padx=10, pady=8)

        # Big primary playback button
        self.btn_play = tk.Button(
            inner_actions, text="▶  Execute Recorded Trajectory",
            bg="#2980B9", fg="white", font=("Segoe UI", 11, "bold"),
            relief="flat", padx=16, pady=8, state="disabled",
            command=self._cmd_execute_trajectory
        )
        self.btn_play.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=4)

        self.btn_pause = tk.Button(
            inner_actions, text="⏸  Pause", bg="#F39C12", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=10, pady=6,
            state="disabled", command=self._cmd_pause
        )
        self.btn_pause.grid(row=0, column=2, sticky="ew", padx=4, pady=4)

        self.btn_resume = tk.Button(
            inner_actions, text="⏯  Resume", bg="#27AE60", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=10, pady=6,
            state="disabled", command=self._cmd_resume
        )
        self.btn_resume.grid(row=0, column=3, sticky="ew", padx=4, pady=4)

        self.btn_stop = tk.Button(
            inner_actions, text="⏹  Stop Replay", bg="#C0392B", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=10, pady=6,
            state="disabled", command=self._cmd_stop
        )
        self.btn_stop.grid(row=0, column=4, sticky="ew", padx=4, pady=4)

        self.btn_move_start = tk.Button(
            inner_actions, text="📍 Move to Trajectory Start", bg="#34495E", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=8, pady=4,
            state="disabled", command=self._cmd_move_to_start
        )
        self.btn_move_start.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=4)

        # Drag-Teaching Control Buttons
        teach_box = ttk.Frame(inner_actions)
        teach_box.grid(row=1, column=2, columnspan=3, sticky="e", pady=4)

        self.btn_record_start = tk.Button(
            teach_box, text="🔴 Start Teaching Record", bg="#8E44AD", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=8, pady=4,
            state="disabled", command=self._cmd_start_teach_record
        )
        self.btn_record_start.pack(side="left", padx=3)

        self.btn_record_stop = tk.Button(
            teach_box, text="⏹ Stop Record", bg="#7F8C8D", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=8, pady=4,
            state="disabled", command=self._cmd_stop_teach_record
        )
        self.btn_record_stop.pack(side="left", padx=3)

        # Power & Emergency Stop Frame
        power_frame = ttk.LabelFrame(self, text="Robot Arm Power & Safety", style="Section.TLabelframe")
        power_frame.pack(fill="x", padx=16, pady=6)

        inner_pwr = ttk.Frame(power_frame)
        inner_pwr.pack(fill="x", padx=10, pady=8)

        self.btn_enable = tk.Button(
            inner_pwr, text="⚡ Enable Motors", bg="#16A085", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_enable_arm
        )
        self.btn_enable.pack(side="left", padx=4)

        self.btn_disable = tk.Button(
            inner_pwr, text="Disable Motors", bg="#95A5A6", fg="white",
            font=("Segoe UI", 9), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_disable_arm
        )
        self.btn_disable.pack(side="left", padx=4)

        self.btn_estop = tk.Button(
            inner_pwr, text="🛑 EMERGENCY STOP", bg="#D63031", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=18, pady=4,
            state="disabled", command=self._cmd_estop
        )
        self.btn_estop.pack(side="right", padx=4)

        # Console / Event Log
        log_frame = ttk.LabelFrame(self, text="Activity & Event Log", style="Section.TLabelframe")
        log_frame.pack(fill="both", expand=True, padx=16, pady=(6, 12))

        self.log_text = tk.Text(log_frame, height=8, bg="#1E1E1E", fg="#D4D4D4", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, padx=6, pady=6)

        self._log("System initialized. Ready to connect.")

    def _log(self, text: str):
        now_str = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{now_str}] {text}\n")
        self.log_text.see(tk.END)

    def _toggle_connection(self):
        if self.controller and self.controller.is_connected():
            # Disconnect
            self._polling_active = False
            self.controller.disconnect()
            self.controller = None
            self.btn_connect.config(text="Connect", bg="#27AE60")
            self.lbl_conn_status.config(text="● Disconnected", fg="#E74C3C")
            self._set_controls_state("disabled")
            self._log("Disconnected from Piper arm.")
        else:
            # Connect
            iface = self.iface_combo.get().strip()
            channel = self.channel_entry.get().strip()
            mock = self.sim_var.get()

            self._log(f"Initiating connection (interface={iface}, channel={channel}, mock={mock})...")
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
                self._log("Connection established successfully.")
                self._start_polling()
            else:
                self.controller = None
                self._log("Connection failed. Check USB-CAN adapter, cables, and 24V power.")
                messagebox.showerror("Connection Failed", "Could not establish CAN communication with Piper.\nCheck hardware connection or enable Simulation Mode.")

    def _set_controls_state(self, state: str):
        for btn in [
            self.btn_play, self.btn_pause, self.btn_resume, self.btn_stop,
            self.btn_move_start, self.btn_record_start, self.btn_record_stop,
            self.btn_enable, self.btn_disable, self.btn_estop
        ]:
            btn.config(state=state)

    def _start_polling(self):
        self._polling_active = True

        def _poll():
            while self._polling_active and self.controller and self.controller.is_connected():
                try:
                    st = self.controller.get_status()
                    joints = self.controller.get_joint_angles()
                    grp = self.controller.get_gripper_stroke()

                    self.after(0, self._update_telemetry_ui, st, joints, grp)
                except Exception as e:
                    pass
                time.sleep(0.1)

        threading.Thread(target=_poll, daemon=True).start()

    def _update_telemetry_ui(self, st: dict, joints: list, grp: float):
        self.lbl_mode.config(text=st["ctrl_mode_name"])
        self.lbl_state.config(text=st["arm_status_name"])
        self.lbl_fps.config(text=f"{st['fps']:.1f} Hz")
        self.lbl_gripper.config(text=f"Gripper Stroke: {grp:.1f} mm")

        # Color highlight arm status if executing trajectory
        if st["is_executing"]:
            self.lbl_state.config(foreground="#E67E22")  # Orange warning/active
        elif st["is_recording"]:
            self.lbl_state.config(foreground="#8E44AD")  # Purple recording
        else:
            self.lbl_state.config(foreground="#27AE60")  # Green normal

        for i, val in enumerate(joints[:6]):
            self.joint_labels[i].config(text=f"J{i+1}: {val:6.1f}°")

    # Command Handlers
    def _cmd_execute_trajectory(self):
        self._log("Triggering: EXECUTE TAUGHT TRAJECTORY (Double-tap substitute)")
        if self.controller:
            self.controller.execute_taught_trajectory()

    def _cmd_pause(self):
        self._log("Triggering: PAUSE TRAJECTORY")
        if self.controller:
            self.controller.pause_trajectory()

    def _cmd_resume(self):
        self._log("Triggering: RESUME TRAJECTORY")
        if self.controller:
            self.controller.resume_trajectory()

    def _cmd_stop(self):
        self._log("Triggering: STOP TRAJECTORY")
        if self.controller:
            self.controller.stop_trajectory()

    def _cmd_move_to_start(self):
        self._log("Triggering: MOVE TO TRAJECTORY START")
        if self.controller:
            self.controller.move_to_trajectory_start()

    def _cmd_start_teach_record(self):
        self._log("Triggering: START TEACHING RECORD")
        if self.controller:
            self.controller.start_teaching_record()

    def _cmd_stop_teach_record(self):
        self._log("Triggering: STOP TEACHING RECORD")
        if self.controller:
            self.controller.stop_teaching_record()

    def _cmd_enable_arm(self):
        self._log("Triggering: ENABLE ARM MOTORS")
        if self.controller:
            self.controller.enable_arm()

    def _cmd_disable_arm(self):
        self._log("Triggering: DISABLE ARM MOTORS")
        if self.controller:
            self.controller.disable_arm()

    def _cmd_estop(self):
        self._log("Triggering: EMERGENCY STOP")
        if self.controller:
            self.controller.emergency_stop()


def main():
    app = PiperGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
