"""Piper Robot Arm Command-Line Interface (CLI).

Provides terminal commands and an interactive text console to test and control
the Piper arm without needing a GUI:
  - Hardware check
  - Passive CAN sniffer
  - Status display
  - Trajectory replay (double-tap replacement)
  - Interactive control console
"""

import sys
import time
import argparse

from piper_controller import PiperRobotController
from hardware_check import check_level_1_hardware, check_level_2_can_init
from can_monitor import start_monitor
from config import ArmStatus, ControlMode


def interactive_menu(controller: PiperRobotController):
    """Run interactive text console for controlling the robot."""
    print("\n" + "=" * 60)
    print(" PIPER ROBOT INTERACTIVE CLI CONTROLLER")
    print("=" * 60)

    try:
        while True:
            st = controller.get_status()
            joints = controller.get_joint_angles()
            grp = controller.get_gripper_stroke()

            print("\n" + "-" * 60)
            print(f" Mode  : {st['ctrl_mode_name']}")
            print(f" Status: {st['arm_status_name']}")
            print(f" Joints: " + " | ".join(f"J{i+1}: {a:.1f}°" for i, a in enumerate(joints)))
            print(f" Gripper Stroke: {grp:.1f} mm")
            print("-" * 60)
            print(" Commands:")
            print("  [1] ▶  Execute Recorded Trajectory (Double-Tap Replay)")
            print("  [2] ⏸  Pause Trajectory")
            print("  [3] ⏯  Resume Trajectory")
            print("  [4] ⏹  Stop Trajectory")
            print("  [5] 📍 Move to Trajectory Start Point")
            print("  [6] ⚡ Enable All Motors")
            print("  [7] 🔒 Disable All Motors")
            print("  [8] 🔴 Start Teaching Recording")
            print("  [9] ⏹  Stop Teaching Recording")
            print("  [0] 🛑 EMERGENCY STOP")
            print("  [r] 🔄 Refresh Telemetry")
            print("  [q] ❌ Disconnect & Exit")
            print("-" * 60)

            choice = input("Enter choice: ").strip().lower()
            if choice == "1":
                controller.execute_taught_trajectory()
            elif choice == "2":
                controller.pause_trajectory()
            elif choice == "3":
                controller.resume_trajectory()
            elif choice == "4":
                controller.stop_trajectory()
            elif choice == "5":
                controller.move_to_trajectory_start()
            elif choice == "6":
                controller.enable_arm()
            elif choice == "7":
                controller.disable_arm()
            elif choice == "8":
                controller.start_teaching_record()
            elif choice == "9":
                controller.stop_teaching_record()
            elif choice == "0":
                controller.emergency_stop()
            elif choice == "r":
                continue
            elif choice == "q":
                print("\n[INFO] Exiting interactive console...")
                break
            else:
                print("[!] Invalid command.")
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        controller.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Piper Robot Arm CLI Suite")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface (agx_cando, slcan, socketcan)")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel index or port (0, COM3, can0)")
    parser.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate (default: 1000000)")
    parser.add_argument("--mock", action="store_true", help="Run with simulated robot state")

    subparsers = parser.add_subparsers(dest="command", help="Available subcommands")

    # check command
    subparsers.add_parser("check", help="Level 1 & 2: Check USB/CAN hardware and communication")

    # monitor command
    p_mon = subparsers.add_parser("monitor", help="Level 3: Passive CAN traffic monitor / sniffer")
    p_mon.add_argument("--log", type=str, default=None, help="Save frames to CSV")
    p_mon.add_argument("--status-only", action="store_true", help="Filter for 0x2A1 status only")

    # status command
    subparsers.add_parser("status", help="Level 4 & 5: Read and print live robot telemetry")

    # play command
    subparsers.add_parser("play", help="Level 6: Execute recorded trajectory (double-tap replacement)")

    # pause command
    subparsers.add_parser("pause", help="Pause executing trajectory")

    # resume command
    subparsers.add_parser("resume", help="Resume paused trajectory")

    # stop command
    subparsers.add_parser("stop", help="Stop/terminate trajectory playback")

    # enable / disable
    subparsers.add_parser("enable", help="Enable all motors")
    subparsers.add_parser("disable", help="Disable all motors")

    # interactive command
    subparsers.add_parser("interactive", help="Start interactive terminal control menu")

    args = parser.parse_args()

    if args.command == "check":
        sys.exit(check_level_1_hardware())

    elif args.command == "monitor":
        start_monitor(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
            log_csv=args.log,
            filter_status_only=args.status_only,
            mock=args.mock,
        )

    else:
        # Commands requiring controller connection
        ctrl = PiperRobotController(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
            mock=args.mock,
        )

        if not ctrl.connect(timeout=2.5):
            print("[ERROR] Could not connect to Piper. Try running 'python piper_cli.py check' or use --mock.")
            sys.exit(1)

        if args.command == "status":
            st = ctrl.get_status()
            joints = ctrl.get_joint_angles()
            grp = ctrl.get_gripper_stroke()
            print("\n--- Live Robot Status ---")
            print(f"Connected : {st['connected']}")
            print(f"Mode      : {st['ctrl_mode_name']}")
            print(f"Arm Status: {st['arm_status_name']}")
            print(f"FPS       : {st['fps']:.1f}")
            print(f"Joints    : {[round(j, 2) for j in joints]}")
            print(f"Gripper   : {grp:.2f} mm")
            ctrl.disconnect()

        elif args.command == "play":
            print("[+] Triggering trajectory execution via CAN...")
            ctrl.execute_taught_trajectory()
            ctrl.wait_for_trajectory_completion(timeout=60.0)
            ctrl.disconnect()

        elif args.command == "pause":
            ctrl.pause_trajectory()
            ctrl.disconnect()

        elif args.command == "resume":
            ctrl.resume_trajectory()
            ctrl.disconnect()

        elif args.command == "stop":
            ctrl.stop_trajectory()
            ctrl.disconnect()

        elif args.command == "enable":
            ctrl.enable_arm()
            ctrl.disconnect()

        elif args.command == "disable":
            ctrl.disable_arm()
            ctrl.disconnect()

        elif args.command == "interactive":
            interactive_menu(ctrl)

        else:
            parser.print_help()
            ctrl.disconnect()


if __name__ == "__main__":
    main()
