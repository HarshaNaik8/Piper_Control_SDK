"""Piper Robot Controller Core API.

Encapsulates the AgileX Piper SDK (C_PiperInterface_V2) with Windows agx_cando support.
Provides high-level programmatic control to:
  1. Trigger taught trajectory replay (the software replacement for physical double-tap).
  2. Monitor live robot status, joint angles, and gripper states.
  3. Pause, resume, and terminate replay operations.
  4. Block or notify on trajectory completion for automated dataset collection pipelines.
  5. Fallback simulation/mock mode for offline testing.
"""

import sys
import time
import threading
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
    """High-level controller for the AgileX Piper 6-DOF Robot Arm."""

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

        # Simulated state variables for mock mode
        self._mock_ctrl_mode = ControlMode.CAN_COMMAND
        self._mock_arm_status = ArmStatus.NORMAL
        self._mock_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._mock_gripper = 50.0  # mm
        self._mock_trajectory_thread: Optional[threading.Thread] = None

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
                raise RuntimeError("piper_sdk is not installed. Install via pip install piper_sdk")

            print(f"[Piper] Initializing CAN connection (interface='{self.interface}', channel='{self.channel}', bitrate={self.bitrate})...")
            try:
                # Do NOT auto-init socketcan because Windows requires agx_cando or slcan
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

            # Wait briefly to confirm CAN frame reception
            start_t = time.time()
            while time.time() - start_t < timeout:
                if self.piper.get_connect_status():
                    self._connected = True
                    print("[Piper] Connected and receiving CAN telemetry.")
                    return True
                time.sleep(0.1)

            print("[WARN] Connection initiated, but no telemetry frames received yet. Please check 24V power.")
            self._connected = True
            return True

    def disconnect(self) -> None:
        """Disconnect and clean up CAN bus."""
        with self._lock:
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
                "is_recording": status_code == ArmStatus.TEACH_RECORDING,
                "is_executing": status_code == ArmStatus.TEACH_EXECUTING,
                "is_paused": status_code == ArmStatus.TEACH_PAUSED,
                "is_normal": status_code == ArmStatus.NORMAL,
                "fps": 50.0,
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
            "is_recording": status_code == ArmStatus.TEACH_RECORDING,
            "is_executing": status_code == ArmStatus.TEACH_EXECUTING,
            "is_paused": status_code == ArmStatus.TEACH_PAUSED,
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
            # Angles are reported in 0.001 deg or millirad depending on firmware configuration
            # In piper_sdk: joint_1 through joint_6 / 1000.0 gives degrees
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

    # -------------------------------------------------------------------------
    # Drag-Teaching Trajectory Playback Functions (Replaces Physical Button)
    # -------------------------------------------------------------------------

    def execute_taught_trajectory(self) -> bool:
        """Trigger execution/playback of recorded trajectory.

        This is the direct software equivalent of double-tapping the physical
        teach button between J5 and J6.
        CAN ID: 0x150, grag_teach_ctrl: 0x03.
        """
        print("[Piper] Command: EXECUTE TAUGHT TRAJECTORY (Replaying motion...)")
        if self.mock:
            self._mock_arm_status = ArmStatus.TEACH_EXECUTING
            def _sim_run():
                print("[MOCK] Trajectory replay running for 4 seconds...")
                time.sleep(4.0)
                self._mock_arm_status = ArmStatus.NORMAL
                print("[MOCK] Trajectory replay finished.")
            threading.Thread(target=_sim_run, daemon=True).start()
            return True

        if not self.is_connected():
            print("[ERROR] Robot not connected.")
            return False

        try:
            # grag_teach_ctrl = 0x03 -> Execute taught trajectory
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.EXECUTE_TRAJECTORY)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send execute command: {e}")
            return False

    def pause_trajectory(self) -> bool:
        """Pause playback of currently running trajectory (0x150, 0x04)."""
        print("[Piper] Command: PAUSE TRAJECTORY")
        if self.mock:
            self._mock_arm_status = ArmStatus.TEACH_PAUSED
            return True

        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.PAUSE)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send pause command: {e}")
            return False

    def resume_trajectory(self) -> bool:
        """Resume playback of paused trajectory (0x150, 0x05)."""
        print("[Piper] Command: RESUME TRAJECTORY")
        if self.mock:
            self._mock_arm_status = ArmStatus.TEACH_EXECUTING
            return True

        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.RESUME)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send resume command: {e}")
            return False

    def stop_trajectory(self) -> bool:
        """Terminate / stop trajectory playback immediately (0x150, 0x06)."""
        print("[Piper] Command: TERMINATE TRAJECTORY")
        if self.mock:
            self._mock_arm_status = ArmStatus.NORMAL
            return True

        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.TERMINATE)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send stop command: {e}")
            return False

    def move_to_trajectory_start(self) -> bool:
        """Move arm to the start point of the recorded trajectory (0x150, 0x07)."""
        print("[Piper] Command: MOVE TO TRAJECTORY START")
        if self.mock:
            print("[MOCK] Arm moved to trajectory start.")
            return True

        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.MOVE_TO_START)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send move to start command: {e}")
            return False

    def start_teaching_record(self) -> bool:
        """Software trigger to enter drag-teach recording mode (0x150, 0x01)."""
        print("[Piper] Command: START TEACHING RECORD")
        if self.mock:
            self._mock_arm_status = ArmStatus.TEACH_RECORDING
            return True

        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.START_RECORD)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send start recording command: {e}")
            return False

    def stop_teaching_record(self) -> bool:
        """Software trigger to exit drag-teach recording mode (0x150, 0x02)."""
        print("[Piper] Command: STOP TEACHING RECORD")
        if self.mock:
            self._mock_arm_status = ArmStatus.NORMAL
            return True

        if not self.is_connected():
            return False
        try:
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.STOP_RECORD)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send stop recording command: {e}")
            return False

    def enable_arm(self) -> bool:
        """Enable all motors on the arm (CAN ID: 0x471, motor=7, enable=0x02)."""
        print("[Piper] Command: ENABLE ALL MOTORS")
        if self.mock:
            print("[MOCK] All motors enabled.")
            return True
        if not self.is_connected():
            return False
        try:
            self.piper.EnableArm(7, 0x02)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to enable arm: {e}")
            return False

    def disable_arm(self) -> bool:
        """Disable all motors on the arm (CAN ID: 0x471, motor=7, enable=0x01)."""
        print("[Piper] Command: DISABLE ALL MOTORS")
        if self.mock:
            print("[MOCK] All motors disabled.")
            return True
        if not self.is_connected():
            return False
        try:
            self.piper.DisableArm(7, 0x01)
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

    def wait_for_trajectory_completion(
        self,
        timeout: float = 60.0,
        poll_interval: float = 0.1,
        on_progress: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> bool:
        """Synchronously wait until trajectory replay begins and then completes.

        Crucial for dataset collection loops: allows waiting for the physical
        movement to conclude before triggering camera saves or the next episode.

        Returns True if motion finished normally, False if timed out or interrupted.
        """
        print(f"[Piper] Waiting for trajectory completion (timeout={timeout}s)...")
        start_time = time.time()
        has_started_execution = False

        # First wait briefly for arm to acknowledge playback mode (arm_status == 0x0C)
        while time.time() - start_time < timeout:
            status = self.get_status()
            if on_progress:
                on_progress(status)

            if status["is_executing"]:
                has_started_execution = True
                break

            time.sleep(poll_interval)

        if not has_started_execution:
            print("[WARN] Robot did not transition into playback status (0x0C) within initial window.")

        # Next wait for arm to finish execution and return to normal/idle status
        while time.time() - start_time < timeout:
            status = self.get_status()
            if on_progress:
                on_progress(status)

            if has_started_execution and (not status["is_executing"]):
                print("[Piper] Trajectory playback completed successfully.")
                return True

            if status["arm_status"] in (ArmStatus.EMERGENCY_STOP, ArmStatus.COLLISION_DETECTED):
                print(f"[ERROR] Trajectory playback stopped due to error: {status['arm_status_name']}")
                return False

            time.sleep(poll_interval)

        print("[WARN] Trajectory wait timed out.")
        return False
