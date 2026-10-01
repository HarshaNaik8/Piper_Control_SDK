"""Piper Robot Controller Core API.

Encapsulates the AgileX Piper SDK (C_PiperInterface_V2) with Windows agx_cando support.
Provides complete programmatic control for dataset collection and demonstration:
  1. Complete Power Control (Enable all motors with 0xFF, Disable, Emergency Stop).
  2. Gripper Freedom (Loose mode for hand-teaching, Stiff mode, Position Control).
  3. Firmware Trajectory Replay Trigger (CAN ID: 0x150, grag_teach_ctrl=0x03).
  4. Software Trajectory Recorder & Mimic Player (Samples & replays 50Hz joint trajectories).
  5. Simulation / Mock Mode for offline development and testing.
"""

import os
import sys
import time
import json
import threading
from datetime import datetime
from typing import Dict, Any, Optional, Callable, List

try:
    from piper_sdk import C_PiperInterface_V2
    PIPER_SDK_AVAILABLE = True
except ImportError:
    PIPER_SDK_AVAILABLE = False

from config import (
    CANConfig,
    DragTeachCmd,
    ControlMode,
    ArmStatus,
    CONTROL_MODE_NAMES,
    ARM_STATUS_NAMES,
    DRAG_TEACH_CMD_NAMES,
)


class PiperRobotController:
    """High-level controller for the AgileX Piper 6-DOF Robot Arm and Gripper."""

    def __init__(
        self,
        interface: str = "agx_cando",
        channel: str = "0",
        bitrate: int = 1000000,
        mock: bool = False,
    ):
        self.interface = interface
        self.channel = str(channel)
        self.bitrate = bitrate
        self.mock = mock

        self.piper: Optional[C_PiperInterface_V2] = None
        self._connected = False
        self._lock = threading.Lock()

        # Software trajectory recording state
        self._is_recording_software = False
        self._recorded_trajectory: List[Dict[str, Any]] = []
        self._record_thread: Optional[threading.Thread] = None

        # Software trajectory playback state
        self._is_replaying_software = False
        self._replay_paused = False
        self._replay_stop_event = threading.Event()
        self._replay_pause_event = threading.Event()
        self._replay_thread: Optional[threading.Thread] = None

        # Simulated state variables for mock mode
        self._mock_ctrl_mode = ControlMode.CAN_COMMAND
        self._mock_arm_status = ArmStatus.NORMAL
        self._mock_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._mock_gripper = 20.0  # mm
        self._mock_gripper_loose = False

    def is_connected(self) -> bool:
        if self.mock:
            return self._connected
        if self.piper is not None:
            return bool(self.piper.get_connect_status())
        return False

    def connect(self, timeout: float = 3.0) -> bool:
        """Establish CAN connection and initialize Piper arm interface."""
        with self._lock:
            if self._connected:
                return True

            if self.mock:
                print(f"[MOCK] Connecting to simulated Piper on {self.interface}:{self.channel}...")
                time.sleep(0.3)
                self._connected = True
                print("[MOCK] Connected successfully.")
                return True

            if not PIPER_SDK_AVAILABLE:
                raise RuntimeError("piper_sdk is not installed. Run: pip install piper_sdk")

            print(f"[Piper] Initializing CAN connection (interface='{self.interface}', channel='{self.channel}', bitrate={self.bitrate})...")
            try:
                self.piper = C_PiperInterface_V2(
                    can_name=self.channel,
                    judge_flag=False,
                    can_auto_init=False,
                )
                self.piper.CreateCanBus(
                    can_name=self.channel,
                    bustype=self.interface,
                    expected_bitrate=self.bitrate,
                    judge_flag=False,
                )
                self.piper.ConnectPort(
                    can_init=False,
                    piper_init=True,
                    start_thread=True,
                )
            except Exception as e:
                print(f"[ERROR] Failed to connect to Piper arm: {e}")
                self.piper = None
                self._connected = False
                return False

            # Wait to confirm CAN telemetry stream
            start_t = time.time()
            while time.time() - start_t < timeout:
                if self.piper.get_connect_status():
                    self._connected = True
                    print("[Piper] Connected and receiving CAN telemetry.")
                    return True
                time.sleep(0.1)

            print("[WARN] CAN connection initialized; waiting for frames.")
            self._connected = True
            return True

    def disconnect(self) -> None:
        """Disconnect and clean up CAN bus."""
        with self._lock:
            # Stop any ongoing recording or replay
            self.stop_software_recording()
            self.stop_software_replay()

            if self.mock:
                self._connected = False
                print("[MOCK] Disconnected.")
                return

            if self.piper is not None:
                try:
                    self.piper.DisconnectPort()
                except Exception as e:
                    print(f"[WARN] Error during DisconnectPort: {e}")
                self.piper = None
            self._connected = False
            print("[Piper] CAN interface closed.")

    # -------------------------------------------------------------------------
    # Telemetry Reading
    # -------------------------------------------------------------------------

    def get_status(self) -> Dict[str, Any]:
        """Fetch current operational status of the robotic arm."""
        if not self.is_connected():
            return {
                "connected": False,
                "ctrl_mode": 0,
                "ctrl_mode_name": "Disconnected",
                "arm_status": 0,
                "arm_status_name": "Disconnected",
                "is_recording": False,
                "is_executing": False,
                "is_paused": False,
                "is_normal": False,
                "fps": 0.0,
            }

        if self.mock:
            status_code = self._mock_arm_status
            mode_code = self._mock_ctrl_mode
            return {
                "connected": True,
                "ctrl_mode": mode_code,
                "ctrl_mode_name": CONTROL_MODE_NAMES.get(mode_code, f"0x{mode_code:02X}"),
                "arm_status": status_code,
                "arm_status_name": ARM_STATUS_NAMES.get(status_code, f"0x{status_code:02X}"),
                "is_recording": status_code == ArmStatus.TEACH_RECORDING or self._is_recording_software,
                "is_executing": status_code == ArmStatus.TEACH_EXECUTING or self._is_replaying_software,
                "is_paused": status_code == ArmStatus.TEACH_PAUSED or self._replay_paused,
                "is_normal": status_code == ArmStatus.NORMAL,
                "fps": 210.0,
            }

        arm_st = self.piper.GetArmStatus()
        status_code = getattr(arm_st.arm_status, "arm_status", 0)
        mode_code = getattr(arm_st.arm_status, "ctrl_mode", 0)
        fps = getattr(arm_st, "Hz", 0.0)

        return {
            "connected": True,
            "ctrl_mode": mode_code,
            "ctrl_mode_name": CONTROL_MODE_NAMES.get(mode_code, f"0x{mode_code:02X}"),
            "arm_status": status_code,
            "arm_status_name": ARM_STATUS_NAMES.get(status_code, f"0x{status_code:02X}"),
            "is_recording": status_code == ArmStatus.TEACH_RECORDING or self._is_recording_software,
            "is_executing": status_code == ArmStatus.TEACH_EXECUTING or self._is_replaying_software,
            "is_paused": status_code == ArmStatus.TEACH_PAUSED or self._replay_paused,
            "is_normal": status_code == ArmStatus.NORMAL,
            "fps": fps,
        }

    def get_joint_angles(self) -> List[float]:
        """Get 6 joint angles in degrees [J1, J2, J3, J4, J5, J6]."""
        if not self.is_connected():
            return [0.0] * 6

        if self.mock:
            return list(self._mock_joints)

        try:
            joints_msg = self.piper.GetArmJointMsgs()
            j1 = getattr(joints_msg.joint_state, "joint_1", 0) / 1000.0
            j2 = getattr(joints_msg.joint_state, "joint_2", 0) / 1000.0
            j3 = getattr(joints_msg.joint_state, "joint_3", 0) / 1000.0
            j4 = getattr(joints_msg.joint_state, "joint_4", 0) / 1000.0
            j5 = getattr(joints_msg.joint_state, "joint_5", 0) / 1000.0
            j6 = getattr(joints_msg.joint_state, "joint_6", 0) / 1000.0
            return [j1, j2, j3, j4, j5, j6]
        except Exception:
            return [0.0] * 6

    def get_gripper_stroke(self) -> float:
        """Get gripper opening stroke in millimeters."""
        if not self.is_connected():
            return 0.0

        if self.mock:
            return self._mock_gripper

        try:
            grp = self.piper.GetArmGripperMsgs()
            return getattr(grp.gripper_state, "grippers_angle", 0) / 1000.0
        except Exception:
            return 0.0

    def get_motor_enable_status(self) -> List[bool]:
        """Get enable status of motors [J1..J6]."""
        if not self.is_connected() or self.mock:
            return [True] * 6
        try:
            return self.piper.GetArmEnableStatus()
        except Exception:
            return [False] * 6

    # -------------------------------------------------------------------------
    # Arm Power & Motors Control (Debugged & Fixed)
    # -------------------------------------------------------------------------

    def enable_arm(self) -> bool:
        """Enable all motors on the arm (makes robot STIFF / position-locked).

        Uses 0xFF to target ALL motors (Joints 1-6 AND Gripper).
        1. Resumes from emergency stop (0x150, emergency_stop=0x02).
        2. Sets CAN command mode (0x151, ctrl_mode=0x01, move_mode=0x01).
        3. Enables all motors with holding torque (0x471, motor_num=0xFF, enable=0x02).
        """
        print("[Piper] Command: ENABLE ALL MOTORS (Stiff / Holding Position)")
        if self.mock:
            self._mock_arm_status = ArmStatus.NORMAL
            print("[MOCK] All motors enabled and stiff.")
            return True
        if not self.is_connected():
            return False
        try:
            # 1. Clear emergency stop
            self.piper.EmergencyStop(0x02)
            time.sleep(0.05)
            # 2. Set CAN command mode
            self.piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50)
            time.sleep(0.05)
            # 3. Enable ALL motors (0xFF = joints 1..6 + gripper)
            self.piper.EnableArm(0xFF, 0x02)
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to enable arm: {e}")
            return False

    def disable_arm(self) -> bool:
        """Disable all motors on the arm (UNPOWERED / DEAD WEIGHT).

        Uses 0xFF to cut torque on ALL motors (Joints 1-6 AND Gripper).
        """
        print("[Piper] Command: DISABLE ALL MOTORS (Unpowered / Dead Weight)")
        if self.mock:
            print("[MOCK] All motors disabled.")
            return True
        if not self.is_connected():
            return False
        try:
            # 0xFF = All motors
            self.piper.DisableArm(0xFF, 0x01)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to disable arm: {e}")
            return False

    def emergency_stop(self) -> bool:
        """Trigger rapid emergency stop (0x150, emergency_stop=0x01)."""
        print("[Piper] !!! EMERGENCY STOP TRIGGERED !!!")
        if self.mock:
            self._mock_arm_status = ArmStatus.EMERGENCY_STOP
            return True
        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0x01, track_ctrl=0, grag_teach_ctrl=0)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to trigger emergency stop: {e}")
            return False

    # -------------------------------------------------------------------------
    # Gripper Freedom & Control (Solving Stiff Gripper during Teaching)
    # -------------------------------------------------------------------------

    def make_gripper_loose(self) -> bool:
        """Make gripper LOOSE so user can easily open/close fingers by hand.

        Disables gripper motor (motor_num=7, enable_flag=0x01) and sets gripper_code=0x00.
        """
        print("[Piper] Gripper: LOOSE MODE (Movable by hand during teaching)")
        if self.mock:
            self._mock_gripper_loose = True
            return True
        if not self.is_connected():
            return False
        try:
            # Motor 7 is the gripper motor
            self.piper.DisableArm(7, 0x01)
            time.sleep(0.02)
            self.piper.GripperCtrl(gripper_angle=0, gripper_effort=0, gripper_code=0x00, set_zero=0)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to make gripper loose: {e}")
            return False

    def make_gripper_stiff(self) -> bool:
        """Make gripper STIFF with active holding torque (motor_num=7, enable_flag=0x02)."""
        print("[Piper] Gripper: STIFF MODE (Holding position)")
        if self.mock:
            self._mock_gripper_loose = False
            return True
        if not self.is_connected():
            return False
        try:
            self.piper.EnableArm(7, 0x02)
            time.sleep(0.02)
            self.piper.GripperCtrl(gripper_angle=int(self.get_gripper_stroke() * 1000), gripper_effort=1000, gripper_code=0x01, set_zero=0)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to make gripper stiff: {e}")
            return False

    def set_gripper_stroke(self, stroke_mm: float, effort: int = 1000) -> bool:
        """Set gripper opening stroke in millimeters (0.0 to 70.0 mm)."""
        if self.mock:
            self._mock_gripper = stroke_mm
            return True
        if not self.is_connected():
            return False
        try:
            angle_raw = int(stroke_mm * 1000)
            self.piper.GripperCtrl(gripper_angle=angle_raw, gripper_effort=effort, gripper_code=0x01, set_zero=0)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to set gripper stroke: {e}")
            return False

    # -------------------------------------------------------------------------
    # Software Trajectory Recorder & Mimic Player (The Dataset Collection Engine)
    # -------------------------------------------------------------------------

    def start_software_recording(self, sample_hz: float = 50.0) -> bool:
        """Start recording live joint angles and gripper positions in software at sample_hz."""
        if self._is_recording_software:
            print("[WARN] Already recording software trajectory.")
            return False

        print(f"[Recorder] Started software trajectory recording at {sample_hz} Hz...")
        self._is_recording_software = True
        self._recorded_trajectory = []
        interval = 1.0 / sample_hz

        def _record_loop():
            start_t = time.time()
            while self._is_recording_software:
                t = time.time() - start_t
                joints = self.get_joint_angles()
                gripper = self.get_gripper_stroke()
                self._recorded_trajectory.append({
                    "time": round(t, 4),
                    "joints": [round(j, 3) for j in joints],
                    "gripper": round(gripper, 3),
                })
                time.sleep(interval)

        self._record_thread = threading.Thread(target=_record_loop, daemon=True)
        self._record_thread.start()
        return True

    def stop_software_recording(self, save_path: Optional[str] = None) -> Optional[str]:
        """Stop software recording and save trajectory to a JSON file."""
        if not self._is_recording_software:
            return None

        self._is_recording_software = False
        if self._record_thread:
            self._record_thread.join(timeout=1.0)
            self._record_thread = None

        num_points = len(self._recorded_trajectory)
        duration = self._recorded_trajectory[-1]["time"] if num_points > 0 else 0.0
        print(f"[Recorder] Stopped recording. Captured {num_points} waypoints ({duration:.2f}s).")

        if not save_path:
            os.makedirs("trajectories", exist_ok=True)
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join("trajectories", f"trajectory_{timestamp_str}.json")

        traj_data = {
            "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "num_points": num_points,
            "duration_sec": duration,
            "waypoints": self._recorded_trajectory,
        }

        try:
            with open(save_path, "w", encoding="utf-8") as f:
                json.dump(traj_data, f, indent=2)
            print(f"[Recorder] Trajectory saved to: {save_path}")
            return save_path
        except Exception as e:
            print(f"[ERROR] Failed to save trajectory file: {e}")
            return None

    def get_recorded_trajectory_info(self) -> Dict[str, Any]:
        """Get live count and duration of currently recording trajectory."""
        num_points = len(self._recorded_trajectory)
        duration = self._recorded_trajectory[-1]["time"] if num_points > 0 else 0.0
        return {
            "is_recording": self._is_recording_software,
            "num_points": num_points,
            "duration_sec": duration,
        }

    def replay_software_trajectory(
        self,
        trajectory: Any,
        speed_factor: float = 1.0,
        loop_count: int = 1,
        on_progress: Optional[Callable[[int, int, Dict[str, Any]], None]] = None,
        blocking: bool = False,
    ) -> bool:
        """Replay recorded trajectory (smoothly reproduces human demonstration).

        Streams JointCtrl (CAN ID: 0x155-0x157) and GripperCtrl (0x159) point-by-point.
        """
        if self._is_replaying_software:
            print("[WARN] Replay already in progress.")
            return False

        # Load waypoints if trajectory is a file path
        waypoints = []
        if isinstance(trajectory, str):
            try:
                with open(trajectory, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    waypoints = data.get("waypoints", [])
            except Exception as e:
                print(f"[ERROR] Failed to load trajectory file '{trajectory}': {e}")
                return False
        elif isinstance(trajectory, list):
            waypoints = trajectory
        elif isinstance(trajectory, dict):
            waypoints = trajectory.get("waypoints", [])

        if not waypoints:
            print("[ERROR] Trajectory has 0 waypoints!")
            return False

        self._replay_stop_event.clear()
        self._replay_pause_event.clear()
        self._is_replaying_software = True
        self._replay_paused = False

        def _replay_worker():
            print(f"[Replayer] Starting playback ({len(waypoints)} points, speed={speed_factor}x, loops={loop_count})...")
            # 1. Switch to CAN control mode and enable all motors
            self.enable_arm()
            time.sleep(0.1)

            try:
                for loop in range(1, loop_count + 1):
                    if self._replay_stop_event.is_set():
                        break

                    # First move to start waypoint smoothly
                    start_pt = waypoints[0]
                    self._send_joint_waypoint(start_pt["joints"], start_pt.get("gripper", 0.0))
                    time.sleep(0.5)

                    prev_t = waypoints[0]["time"]
                    total_pts = len(waypoints)

                    for idx, pt in enumerate(waypoints):
                        if self._replay_stop_event.is_set():
                            print("[Replayer] Replay stopped by user.")
                            break

                        # Handle pause
                        while self._replay_pause_event.is_set():
                            self._replay_paused = True
                            time.sleep(0.05)
                            if self._replay_stop_event.is_set():
                                break
                        self._replay_paused = False

                        # Send joint & gripper positions
                        self._send_joint_waypoint(pt["joints"], pt.get("gripper", 0.0))

                        if on_progress:
                            on_progress(idx + 1, total_pts, pt)

                        # Timing delay to match original demonstration speed
                        dt = (pt["time"] - prev_t) / max(0.1, speed_factor)
                        prev_t = pt["time"]
                        if dt > 0.001:
                            time.sleep(min(dt, 0.1))

                    print(f"[Replayer] Loop {loop}/{loop_count} completed.")
                    if loop < loop_count and not self._replay_stop_event.is_set():
                        time.sleep(1.0)

            finally:
                self._is_replaying_software = False
                self._replay_paused = False
                print("[Replayer] Playback finished.")

        self._replay_thread = threading.Thread(target=_replay_worker, daemon=True)
        self._replay_thread.start()

        if blocking:
            self._replay_thread.join()

        return True

    def _send_joint_waypoint(self, joints: List[float], gripper_mm: float):
        """Send 6-DOF joint target and gripper command."""
        if self.mock:
            self._mock_joints = list(joints)
            self._mock_gripper = gripper_mm
            return

        if not self.is_connected():
            return

        try:
            # JointCtrl expects angles in 0.001 deg (int)
            j1 = int(round(joints[0] * 1000))
            j2 = int(round(joints[1] * 1000))
            j3 = int(round(joints[2] * 1000))
            j4 = int(round(joints[3] * 1000))
            j5 = int(round(joints[4] * 1000))
            j6 = int(round(joints[5] * 1000))
            self.piper.JointCtrl(j1, j2, j3, j4, j5, j6)

            # GripperCtrl expects angle in 0.001 mm
            grp_raw = int(round(gripper_mm * 1000))
            self.piper.GripperCtrl(gripper_angle=grp_raw, gripper_effort=1000, gripper_code=0x01, set_zero=0)
        except Exception as e:
            pass

    def pause_software_replay(self):
        """Pause software playback."""
        print("[Replayer] Pausing replay...")
        self._replay_pause_event.set()

    def resume_software_replay(self):
        """Resume software playback."""
        print("[Replayer] Resuming replay...")
        self._replay_pause_event.clear()

    def stop_software_replay(self):
        """Stop software playback immediately."""
        print("[Replayer] Stopping replay...")
        self._replay_stop_event.set()
        self._replay_pause_event.clear()
        if self._replay_thread and self._replay_thread.is_alive():
            self._replay_thread.join(timeout=1.0)
            self._replay_thread = None
        self._is_replaying_software = False

    def is_replaying_software(self) -> bool:
        return self._is_replaying_software

    # -------------------------------------------------------------------------
    # Firmware Trajectory Control (Hardware Button Replay via CAN)
    # -------------------------------------------------------------------------

    def execute_firmware_trajectory(self) -> bool:
        """Trigger replay of the trajectory recorded via the physical green button.

        Sends CAN ID 0x150, grag_teach_ctrl=0x03.
        Note: The trajectory MUST have been recorded using the physical teach button
        (single click -> move -> single click) for the firmware buffer to contain it!
        """
        print("[Piper] Command: EXECUTE FIRMWARE TRAJECTORY (Double-tap substitute, 0x150 grag_teach_ctrl=0x03)")
        if self.mock:
            self._mock_arm_status = ArmStatus.TEACH_EXECUTING
            def _sim_run():
                time.sleep(3.0)
                self._mock_arm_status = ArmStatus.NORMAL
            threading.Thread(target=_sim_run, daemon=True).start()
            return True

        if not self.is_connected():
            return False

        try:
            # Clear e-stop if active
            self.piper.EmergencyStop(0x02)
            time.sleep(0.05)
            # grag_teach_ctrl = 0x03 -> Execute taught trajectory
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.EXECUTE_TRAJECTORY)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send firmware execute command: {e}")
            return False

    def pause_firmware_trajectory(self) -> bool:
        """Pause firmware trajectory replay (0x150, grag_teach_ctrl=0x04)."""
        print("[Piper] Command: PAUSE FIRMWARE TRAJECTORY")
        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.PAUSE)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send pause command: {e}")
            return False

    def resume_firmware_trajectory(self) -> bool:
        """Resume firmware trajectory replay (0x150, grag_teach_ctrl=0x05)."""
        print("[Piper] Command: RESUME FIRMWARE TRAJECTORY")
        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.RESUME)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send resume command: {e}")
            return False

    def stop_firmware_trajectory(self) -> bool:
        """Terminate firmware trajectory replay (0x150, grag_teach_ctrl=0x06)."""
        print("[Piper] Command: TERMINATE FIRMWARE TRAJECTORY")
        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.TERMINATE)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send terminate command: {e}")
            return False

    def move_to_trajectory_start(self) -> bool:
        """Move arm to the start point of the firmware trajectory (0x150, grag_teach_ctrl=0x07)."""
        print("[Piper] Command: MOVE TO TRAJECTORY START")
        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.MOVE_TO_START)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send move to start command: {e}")
            return False
