"""Live Robot Telemetry & State Monitor (Testing Ladder Level 4 & 5).

Continuously polls and prints the Piper's live state without sending motion commands:
  - Joint positions (J1-J6 in degrees)
  - Gripper stroke (mm)
  - Robot operational mode & status
  - Communication rate (FPS)
"""

import sys
import time
import argparse

from piper_controller import PiperRobotController
from config import ARM_STATUS_NAMES, CONTROL_MODE_NAMES


def run_state_monitor(
    interface: str = "agx_cando",
    channel: str = "0",
    rate_hz: float = 5.0,
    mock: bool = False,
) -> None:
    print("=" * 75)
    print(" PIPER ROBOT TELEMETRY & STATE MONITOR (LEVEL 4 & 5)")
    print(f" Interface : {interface} | Channel: {channel}")
    print(f" Polling Rate: {rate_hz} Hz")
    print(" Press Ctrl+C to exit.")
    print("=" * 75)

    controller = PiperRobotController(
        interface=interface,
        channel=channel,
        mock=mock,
    )

    if not controller.connect(timeout=3.0):
        print("[ERROR] Could not connect to Piper. Ensure USB-CAN adapter is plugged in and powered.")
        return

    delay = 1.0 / rate_hz
    try:
        while True:
            st = controller.get_status()
            joints = controller.get_joint_angles()
            gripper = controller.get_gripper_stroke()

            # Format line
            j_str = ", ".join(f"J{i+1}: {deg:6.1f}°" for i, deg in enumerate(joints))
            print(
                f"\r[STATUS] Mode: {st['ctrl_mode_name']:<24s} | "
                f"State: {st['arm_status_name']:<30s} | "
                f"Gripper: {gripper:5.1f}mm | FPS: {st['fps']:4.1f} | {j_str}",
                end="",
                flush=True,
            )
            time.sleep(delay)
    except KeyboardInterrupt:
        print("\n\n[INFO] Exiting telemetry monitor.")
    finally:
        controller.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Piper Live State Monitor")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel")
    parser.add_argument("--rate", type=float, default=5.0, help="Polling rate in Hz")
    parser.add_argument("--mock", action="store_true", help="Run in mock/simulation mode")
    args = parser.parse_args()

    run_state_monitor(
        interface=args.interface,
        channel=args.channel,
        rate_hz=args.rate,
        mock=args.mock,
    )


if __name__ == "__main__":
    main()
