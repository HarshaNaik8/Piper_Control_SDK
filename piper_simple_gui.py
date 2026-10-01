"""Piper Simple & Safe GUI - Dedicated Trajectory Replay & Camera Sync Controller.

Simplicity Wins Over Perfection:
  - Big, unambiguous 'REPLAY TRAJECTORY' button (double-tap replacement).
  - Clean Gripper controls: Enable (Stiff), Disable (Loose).
  - No pause/resume buttons. No complex stroke inputs.
  - Zero dangerous disable toggles for the main arm!
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from typing import Optional

from piper_playback import PiperPlaybackController


class PiperSimpleGUI(tk.Tk):
    """Clean, dedicated Tkinter GUI for Piper Trajectory Playback."""

    def __init__(self):
        super().__init__()
        self.title("Piper Arm Controller - Replay Trigger")
        self.geometry("600x550")
        self.minsize(500, 500)

        self._setup_styles()

        self.controller: Optional[PiperPlaybackController] = None
        self._polling_active = False

        self._build_ui()

    def _setup_styles(self):
        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.style.configure("TFrame", background="#F8F9FA")
        self.style.configure("Header.TLabel", font=("Segoe UI", 13, "bold"), background="#F8F9FA", foreground="#2C3E50")
        self.style.configure("Section.TLabelframe", background="#F8F9FA")
        self.style.configure("Section.TLabelframe.Label", font=("Segoe UI", 10, "bold"), foreground="#34495E")
        self.configure(bg="#F8F9FA")

    def _build_ui(self):
        # 1. Header
        header = ttk.Frame(self)
        header.pack(fill="x", padx=16, pady=(10, 4))

        title_lbl = ttk.Label(header, text="Piper Software Double-Tap Trigger", style="Header.TLabel")
        title_lbl.pack(side="left")

        self.sim_var = tk.BooleanVar(value=False)
        sim_chk = ttk.Checkbutton(header, text="Simulation (Offline)", variable=self.sim_var)
        sim_chk.pack(side="right")

        # 2. CAN Connection Settings
        conn_frame = ttk.LabelFrame(self, text="CAN Connection", style="Section.TLabelframe")
        conn_frame.pack(fill="x", padx=16, pady=4)

        conn_box = ttk.Frame(conn_frame)
        conn_box.pack(fill="x", padx=10, pady=6)

        ttk.Label(conn_box, text="Channel:").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.channel_entry = ttk.Entry(conn_box, width=6)
        self.channel_entry.insert(0, "0")
        self.channel_entry.grid(row=0, column=1, padx=4)

        self.btn_connect = tk.Button(
            conn_box, text="Connect", bg="#27AE60", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=16, pady=3,
            command=self._toggle_connection
        )
        self.btn_connect.grid(row=0, column=2, padx=(16, 6))

        self.lbl_conn_status = tk.Label(
            conn_box, text="● Disconnected", fg="#E74C3C",
            bg="#F8F9FA", font=("Segoe UI", 10, "bold")
        )
        self.lbl_conn_status.grid(row=0, column=3, padx=8)

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

        # 4. Trajectory Replay Control (The Main Goal!)
        action_frame = ttk.LabelFrame(self, text="Trajectory Playback (Double-Tap Replacement)", style="Section.TLabelframe")
        action_frame.pack(fill="x", padx=16, pady=4)

        action_box = ttk.Frame(action_frame)
        action_box.pack(fill="x", padx=10, pady=8)

        # Guidance note
        ttk.Label(
            action_box,
            text="📌 Instruction: Teach the motion using the physical round green button (Single-click -> Move arm -> Single-click).\n"
                 "    Then click the button below to replay on demand with zero human intervention!",
            font=("Segoe UI", 9, "italic"), foreground="#2C3E50"
        ).pack(anchor="w", pady=(0, 8))

        # Big primary playback trigger
        self.btn_replay = tk.Button(
            action_box, text="▶  REPLAY TRAJECTORY (Double-Tap Trigger)",
            bg="#2980B9", fg="white", font=("Segoe UI", 12, "bold"),
            relief="flat", padx=18, pady=10, state="disabled",
            command=self._cmd_trigger_replay
        )
        self.btn_replay.pack(fill="x", pady=4)

        # 5. Safety & Power Controls (Hold vs Dead Weight)
        power_frame = ttk.LabelFrame(self, text="Safety & Power Controls", style="Section.TLabelframe")
        power_frame.pack(fill="x", padx=16, pady=4)

        power_box = ttk.Frame(power_frame)
        power_box.pack(fill="x", padx=10, pady=6)

        self.btn_hold = tk.Button(
            power_box, text="💪 Hold Position (Stiff)", bg="#8E44AD", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=12, pady=6,
            state="disabled", command=self._cmd_hold_stiff
        )
        self.btn_hold.grid(row=0, column=0, padx=4, pady=2, sticky="ew")

        self.btn_stop = tk.Button(
            power_box, text="🛑 Temporary Stop (Dead Weight)", bg="#E67E22", fg="white",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=12, pady=6,
            state="disabled", command=self._cmd_dead_weight
        )
        self.btn_stop.grid(row=0, column=1, padx=10, pady=2, sticky="ew")

        # 6. Safe Gripper Controls (No motor cuts, No jerking)
        grp_frame = ttk.LabelFrame(self, text="Gripper Motor Control", style="Section.TLabelframe")
        grp_frame.pack(fill="x", padx=16, pady=4)

        grp_box = ttk.Frame(grp_frame)
        grp_box.pack(fill="x", padx=10, pady=6)

        self.btn_grp_enable = tk.Button(
            grp_box, text="Enable Gripper (Stiff / Hold)", bg="#16A085", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_enable_gripper
        )
        self.btn_grp_enable.grid(row=0, column=0, padx=4, pady=2)

        self.btn_grp_disable = tk.Button(
            grp_box, text="Disable Gripper (Loose / Movable)", bg="#C0392B", fg="white",
            font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=4,
            state="disabled", command=self._cmd_disable_gripper
        )
        self.btn_grp_disable.grid(row=0, column=1, padx=4, pady=2)

        # 6. Event Console Log
        log_frame = ttk.LabelFrame(self, text="Activity & Event Log", style="Section.TLabelframe")
        log_frame.pack(fill="both", expand=True, padx=16, pady=(4, 10))

        self.log_text = tk.Text(log_frame, height=6, bg="#1E1E1E", fg="#D4D4D4", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, padx=6, pady=4)

        self._log("System initialized. Ready to connect.")

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
            channel = self.channel_entry.get().strip()
            mock = self.sim_var.get()

            self._log(f"Connecting to agx_cando:{channel} (mock={mock})...")
            self.controller = PiperPlaybackController(
                interface="agx_cando",
                channel=channel,
                mock=mock,
            )
            success = self.controller.connect(timeout=2.5)
            if success:
                self.btn_connect.config(text="Disconnect", bg="#E67E22")
                self.lbl_conn_status.config(text="● Connected", fg="#27AE60")
                self._set_controls_state("normal")
                self._log("Connected to Piper arm! Telemetry streaming.")
                self._start_polling()
            else:
                self.controller = None
                self._log("Connection failed. Check USB-CAN adapter, cables, and 24V power.")
                messagebox.showerror("Connection Failed", "Could not connect to Piper. Check hardware or tick Simulation Mode.")

    def _set_controls_state(self, state: str):
        buttons = [self.btn_replay, self.btn_hold, self.btn_stop, self.btn_grp_enable, self.btn_grp_disable]
        for b in buttons:
            b.config(state=state)

    def _start_polling(self):
        self._polling_active = True

        def _poll():
            while self._polling_active and self.controller and self.controller.is_connected():
                try:
                    st = self.controller.get_status()
                    joints = self.controller.get_joint_angles()
                    self.after(0, self._update_ui, st, joints)
                except Exception:
                    pass
                time.sleep(0.1)

        threading.Thread(target=_poll, daemon=True).start()

    def _update_ui(self, st: dict, joints: list):
        self.lbl_mode.config(text=st["ctrl_mode_name"])
        self.lbl_state.config(text=st["arm_status_name"])
        self.lbl_fps.config(text=f"{st['fps']:.1f} Hz")

        # Color highlight
        if st["is_executing"]:
            self.lbl_state.config(foreground="#E67E22")  # Orange playback active
        else:
            self.lbl_state.config(foreground="#27AE60")  # Green idle

        for i, val in enumerate(joints[:6]):
            if i < len(self.joint_labels):
                self.joint_labels[i].config(text=f"J{i+1}: {val:.1f}°")

    def _cmd_trigger_replay(self):
        self._log("Triggering Trajectory Replay (CAN 0x150)...")
        if self.controller:
            def _run():
                success = self.controller.trigger_replay()
                if success:
                    self._log("Replay triggered successfully.")
                else:
                    self._log("Error: Failed to trigger replay.")
            threading.Thread(target=_run, daemon=True).start()

    def _cmd_hold_stiff(self):
        self._log("Applying Hold Position (Stiff)...")
        if self.controller:
            self.controller.set_hold_stiff()

    def _cmd_dead_weight(self):
        self._log("Applying Temporary Stop (Dead Weight)...")
        if self.controller:
            self.controller.set_dead_weight()

    def _cmd_enable_gripper(self):
        self._log("Enabling Gripper (Motor 7)...")
        if self.controller:
            self.controller.enable_gripper()

    def _cmd_disable_gripper(self):
        self._log("Disabling Gripper (Motor 7)...")
        if self.controller:
            self.controller.disable_gripper()


if __name__ == "__main__":
    app = PiperSimpleGUI()
    app.mainloop()
