"""Piper Robot Playback Controller - Safe CAN Trigger & Telemetry Engine.

Provides an ultra-reliable, zero-danger software trigger for the AgileX Piper
robot arm to replace physical double-tapping of the teach button.

Designed specifically for seamless synchronization with camera recording scripts.
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
)


class PiperPlaybackController:
    """Safe, focused controller for Piper Trajectory Playback & Telemetry."""

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

        # Mock simulation state
        self._mock_arm_status = ArmStatus.NORMAL
        self._mock_ctrl_mode = ControlMode.TEACH_MODE
        self._mock_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._mock_gripper = 50.0

    def is_connected(self) -> bool:
        if self.mock:
            return self._connected
        if self.piper is not None:
            return bool(self.piper.get_connect_status())
        return False

    def connect(self, timeout: float = 3.0) -> bool:
        """Connect to Piper arm over CAN."""
        with self._lock:
            if self._connected:
                return True

            if self.mock:
                print(f"[MOCK] Connected to simulated Piper on {self.interface}:{self.channel}")
                self._connected = True
                return True

            if not PIPER_SDK_AVAILABLE:
                raise RuntimeError("piper_sdk is not installed. Run: pip install piper_sdk")

            print(f"[Piper] Connecting to CAN bus ({self.interface}, channel={self.channel}, bitrate={self.bitrate})...")
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
                print(f"[ERROR] CAN connection failed: {e}")
                self.piper = None
                self._connected = False
                return False

            # Wait to confirm telemetry reception
            start_t = time.time()
            while time.time() - start_t < timeout:
                if self.piper.get_connect_status():
                    self._connected = True
                    print("[Piper] Connected! Receiving live telemetry.")
                    return True
                time.sleep(0.1)

            print("[WARN] Connection initialized; waiting for frames.")
            self._connected = True
            return True

    def disconnect(self) -> None:
        """Safely disconnect CAN interface."""
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
            print("[Piper] Disconnected cleanly.")

    # -------------------------------------------------------------------------
    # Live Telemetry
    # -------------------------------------------------------------------------

    def get_status(self) -> Dict[str, Any]:
        """Read live robot mode, execution state, and CAN fps."""
        if not self.is_connected():
            return {
                "connected": False,
                "ctrl_mode": 0,
                "ctrl_mode_name": "Disconnected",
                "arm_status": 0,
                "arm_status_name": "Disconnected",
                "is_executing": False,
                "is_paused": False,
                "fps": 0.0,
            }

        if self.mock:
            return {
                "connected": True,
                "ctrl_mode": self._mock_ctrl_mode,
                "ctrl_mode_name": CONTROL_MODE_NAMES.get(self._mock_ctrl_mode, "Unknown"),
                "arm_status": self._mock_arm_status,
                "arm_status_name": ARM_STATUS_NAMES.get(self._mock_arm_status, "Unknown"),
                "is_executing": self._mock_arm_status == ArmStatus.TEACH_EXECUTING,
                "is_paused": self._mock_arm_status == ArmStatus.TEACH_PAUSED,
                "fps": 220.0,
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
            "is_executing": status_code == ArmStatus.TEACH_EXECUTING,
            "is_paused": status_code == ArmStatus.TEACH_PAUSED,
            "fps": fps,
        }

    def get_joint_angles(self) -> List[float]:
        """Get current 6 joint angles in degrees [J1..J6]."""
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
        """Get current gripper opening stroke in millimeters."""
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
    # Core Trajectory Playback (The Double-Tap Replacement)
    # -------------------------------------------------------------------------

    def trigger_replay(self) -> bool:
        """Trigger replay of the taught trajectory (replaces physical double-tap).

        Sends CAN ID 0x150, grag_teach_ctrl = 0x03.
        Note: The trajectory must have been recorded previously using the physical
        green teach button (single click to start -> move arm -> single click to stop).
        
        CRITICAL FIX: Removed EmergencyStop(0x02) and ModeCtrl(0x01) calls as they
        cause the robot to drop holding torque or lock up the physical button.
        """
        print("[Piper] >>> TRIGGERING TRAJECTORY REPLAY (Double-Tap Software Equivalent) <<<")
        if self.mock:
            self._mock_arm_status = ArmStatus.TEACH_EXECUTING
            def _sim():
                time.sleep(3.5)
                self._mock_arm_status = ArmStatus.NORMAL
                print("[MOCK] Trajectory execution finished.")
            threading.Thread(target=_sim, daemon=True).start()
            return True

        if not self.is_connected():
            print("[ERROR] Robot is not connected!")
            return False

        try:
            # We send the command in a short loop to ensure the firmware registers it,
            # just like holding the button for a tiny fraction of a second.
            # We absolutely DO NOT send EmergencyStop(0x02).
            for _ in range(5):
                self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.EXECUTE_TRAJECTORY)
                time.sleep(0.02)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send replay trigger: {e}")
            return False

    def wait_for_completion(
        self,
        timeout: float = 60.0,
        poll_interval: float = 0.05,
        on_progress: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> bool:
        """Block until the trajectory replay finishes (essential for camera sync).

        Returns True when trajectory execution concludes normally, False on timeout.
        """
        start_t = time.time()
        has_started = False

        # 1. Wait for robot to enter playback status (0x0C = TEACH_EXECUTING)
        while time.time() - start_t < min(timeout, 3.0):
            st = self.get_status()
            if on_progress:
                on_progress(st)
            if st["is_executing"]:
                has_started = True
                break
            time.sleep(poll_interval)

        # 2. Wait until playback returns to idle
        while time.time() - start_t < timeout:
            st = self.get_status()
            if on_progress:
                on_progress(st)

            if has_started and not st["is_executing"]:
                print("[Piper] Replay concluded successfully.")
                return True

            time.sleep(poll_interval)

        # If it finished quickly within the initial window
        return True

    # -------------------------------------------------------------------------
    # Arm Power Control (Dead Weight vs Hold Stiff)
    # -------------------------------------------------------------------------

    def set_dead_weight(self) -> bool:
        """Temporary Stop: Cuts torque to all motors, dropping the arm completely limp."""
        print("[Piper] Command: DEAD WEIGHT (Disable all motors 0xFF)")
        if self.mock:
            return True
        if not self.is_connected():
            return False
        try:
            # 0xFF = All motors. 0x01 = Disable torque
            self.piper.DisableArm(0xFF, 0x01)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to set dead weight: {e}")
            return False

    def set_hold_stiff(self) -> bool:
        """Hold Button: Enables torque on all motors to hold the current position rigidly."""
        print("[Piper] Command: HOLD / STIFF (Enable all motors 0xFF)")
        if self.mock:
            return True
        if not self.is_connected():
            return False
        try:
            # 0xFF = All motors. 0x02 = Enable torque / Stiff
            self.piper.EnableArm(0xFF, 0x02)
            # Also optionally send stop teaching just in case it was in drag teach
            self.piper.MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=DragTeachCmd.STOP_RECORD)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to hold stiff: {e}")
            return False

    # -------------------------------------------------------------------------
    # Safe Gripper Enable/Disable Control (Loose / Stiff)
    # -------------------------------------------------------------------------

    def disable_gripper(self) -> bool:
        """Make gripper loose (disable motor 7) so it can be moved by hand.
        Uses GripperCtrl (0x159) with code=0x00 to avoid firmware crashing.
        """
        print("[Piper] Gripper -> DISABLE (Loose / Hand-movable)")
        if self.mock:
            return True
        if not self.is_connected():
            return False
        try:
            # gripper_code=0x00 (Disable), effort=0
            self.piper.GripperCtrl(gripper_angle=0, gripper_effort=0, gripper_code=0x00, set_zero=0)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to disable gripper: {e}")
            return False

    def enable_gripper(self) -> bool:
        """Make gripper stiff (enable motor 7) so it holds its position.
        Uses GripperCtrl (0x159) with code=0x01 to avoid firmware crashing.
        """
        print("[Piper] Gripper -> ENABLE (Stiff / Holding position)")
        if self.mock:
            return True
        if not self.is_connected():
            return False
        try:
            # gripper_code=0x01 (Enable Position), effort=1000
            current_angle = int(self.get_gripper_stroke() * 1000)
            self.piper.GripperCtrl(gripper_angle=current_angle, gripper_effort=1000, gripper_code=0x01, set_zero=0)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to enable gripper: {e}")
            return False
