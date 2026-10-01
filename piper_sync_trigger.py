"""Piper Sync Trigger Module & CLI for Multi-Camera Dataset Collection.

Designed to be imported directly into external Python automation scripts
(e.g., mentor's multi-phone video recording pipeline) or invoked from the CLI.

Example Mentor Script Integration:
  ```python
  from piper_sync_trigger import PiperSyncTrigger

  robot = PiperSyncTrigger()
  robot.connect()

  for episode in range(10):
      # 1. Start phone video recording
      start_phone_cameras()

      # 2. Trigger robot motion simultaneously (Zero ms delay!)
      robot.replay_and_wait(timeout=60.0)

      # 3. Stop phone video recording, adjust ISO/exposure
      stop_phone_cameras()
      shift_exposure_settings()

  robot.disconnect()
  ```
"""

import sys
import time
import argparse
from typing import Optional, Dict, Any

from piper_playback import PiperPlaybackController


class PiperSyncTrigger:
    """Zero-overhead trigger interface for camera and sensor synchronization."""

    def __init__(
        self,
        interface: str = "agx_cando",
        channel: str = "0",
        bitrate: int = 1000000,
        mock: bool = False,
    ):
        self.controller = PiperPlaybackController(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            mock=mock,
        )

    def connect(self, timeout: float = 3.0) -> bool:
        """Connect to the Piper robot arm."""
        return self.controller.connect(timeout=timeout)

    def disconnect(self) -> None:
        """Disconnect cleanly."""
        self.controller.disconnect()

    def replay(self) -> bool:
        """Trigger trajectory replay immediately without waiting."""
        return self.controller.trigger_replay()

    def wait_until_idle(self, timeout: float = 60.0) -> bool:
        """Block until the trajectory replay finishes."""
        return self.controller.wait_for_completion(timeout=timeout)

    def replay_and_wait(self, timeout: float = 60.0) -> bool:
        """Trigger replay and block until completion (one-line sync)."""
        success = self.controller.trigger_replay()
        if not success:
            return False
        return self.controller.wait_for_completion(timeout=timeout)

    def dead_weight(self) -> bool:
        return self.controller.set_dead_weight()

    def hold_stiff(self) -> bool:
        return self.controller.set_hold_stiff()

    def enable_gripper(self) -> bool:
        return self.controller.enable_gripper()

    def disable_gripper(self) -> bool:
        return self.controller.disable_gripper()

    def get_status(self) -> Dict[str, Any]:
        return self.controller.get_status()

    def get_joints(self):
        return self.controller.get_joint_angles()

    def get_gripper(self) -> float:
        return self.controller.get_gripper_stroke()


def main():
    parser = argparse.ArgumentParser(description="Piper Camera-Sync Replay Trigger")
    parser.add_argument("command", choices=["trigger", "status", "enable-gripper", "disable-gripper", "dead-weight", "hold-stiff", "loop"], help="Action to execute")
    parser.add_argument("--episodes", type=int, default=3, help="Number of replay loops (for 'loop' command)")
    parser.add_argument("--delay", type=float, default=2.0, help="Delay between loops in seconds")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel")
    parser.add_argument("--mock", action="store_true", help="Run in simulation mode")
    args = parser.parse_args()

    trigger = PiperSyncTrigger(
        interface=args.interface,
        channel=args.channel,
        mock=args.mock,
    )

    if not trigger.connect():
        print("[ERROR] Could not connect to Piper robot arm.")
        sys.exit(1)

    try:
        if args.command == "trigger":
            print("[+] Triggering trajectory replay...")
            trigger.replay_and_wait(timeout=60.0)
            print("[SUCCESS] Trajectory replay finished.")

        elif args.command == "status":
            st = trigger.get_status()
            joints = trigger.get_joints()
            grp = trigger.get_gripper()
            print("\n--- Piper Live Status ---")
            print(f"Connected : {st['connected']}")
            print(f"Mode      : {st['ctrl_mode_name']}")
            print(f"Arm Status: {st['arm_status_name']}")
            print(f"FPS       : {st['fps']:.1f}")
            print(f"Joints    : {[round(j, 2) for j in joints]}")
            print(f"Gripper   : {grp:.1f} mm\n")

        elif args.command == "dead-weight":
            print("[+] Temporary Stop -> DEAD WEIGHT...")
            trigger.dead_weight()

        elif args.command == "hold-stiff":
            print("[+] Hold Position -> STIFF...")
            trigger.hold_stiff()

        elif args.command == "enable-gripper":
            print("[+] Enabling Gripper (Holding torque ON)...")
            trigger.enable_gripper()

        elif args.command == "disable-gripper":
            print("[+] Disabling Gripper (Loose/Movable by hand)...")
            trigger.disable_gripper()

        elif args.command == "loop":
            print(f"[+] Starting {args.episodes} automated replay episodes...")
            for ep in range(1, args.episodes + 1):
                print(f"\n--- Episode {ep} / {args.episodes} ---")
                print(f"[Sync] Camera recording triggered.")
                trigger.replay_and_wait(timeout=60.0)
                print(f"[Sync] Episode {ep} completed.")
                if ep < args.episodes:
                    time.sleep(args.delay)
            print("[SUCCESS] All episodes completed.")

    finally:
        trigger.disconnect()


if __name__ == "__main__":
    main()
