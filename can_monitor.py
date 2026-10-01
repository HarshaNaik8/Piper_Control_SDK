"""Live CAN Monitor & Drag-Teach Button Activity Sniffer (Level 3).

Monitors live CAN telemetry from the Piper robot arm to observe state transitions
when the physical teach button between J5 and J6 is single-clicked or double-clicked.
"""

import sys
import time
import csv
import argparse
from datetime import datetime
from typing import Optional

from piper_sdk import C_PiperInterface_V2
from config import CONTROL_MODE_NAMES, ARM_STATUS_NAMES, DRAG_TEACH_CMD_NAMES


def run_monitor(
    interface: str = "agx_cando",
    channel: str = "0",
    bitrate: int = 1000000,
    log_csv: Optional[str] = None,
    mock: bool = False,
):
    print("=" * 75)
    print(" PIPER ROBOT LIVE CAN MONITOR & BUTTON SNIFFER (LEVEL 3)")
    print(f" Interface : {interface} | Channel: {channel} | Bitrate: {bitrate}")
    if log_csv:
        print(f" Logging to: {log_csv}")
    print("=" * 75)
    print("Instructions for physical observation:")
    print("  1. Observe the LED button between J5 and J6 on the Piper arm.")
    print("  2. Single click: Starts/stops drag-teach recording (Solid Green LED).")
    print("  3. Double click: Replays the recorded trajectory (Flashing Green LED).")
    print("  4. Press Ctrl+C in this terminal when finished.")
    print("=" * 75 + "\n")

    csv_file = None
    csv_writer = None
    if log_csv:
        csv_file = open(log_csv, mode="w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Timestamp", "Control_Mode", "Arm_Status", "Teach_Status", "Joint_Angles", "Gripper_mm"])

    if mock:
        print("[INFO] Running in mock/simulation mode...")
        try:
            while True:
                time.sleep(0.5)
                now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{now}] Mode: [CAN Command Mode] | Status: [Normal / Idle] | Joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]", flush=True)
        except KeyboardInterrupt:
            print("\n[INFO] Stopped.")
        return

    piper = C_PiperInterface_V2(can_name=channel, judge_flag=False, can_auto_init=False)
    try:
        piper.CreateCanBus(can_name=channel, bustype=interface, expected_bitrate=bitrate, judge_flag=False)
        piper.ConnectPort(can_init=False, piper_init=True, start_thread=True)
    except Exception as e:
        print(f"[ERROR] Failed to connect: {e}")
        return

    # Wait for telemetry
    time.sleep(0.5)
    if not piper.get_connect_status():
        print("[WARN] Waiting for first CAN frame...")
        for _ in range(20):
            time.sleep(0.1)
            if piper.get_connect_status():
                break

    print("[SUCCESS] Connected to Piper arm! Telemetry streaming active.\n")

    prev_mode = None
    prev_status = None
    prev_teach = None

    try:
        while True:
            arm_st = piper.GetArmStatus()
            joints_msg = piper.GetArmJointMsgs()
            grp_msg = piper.GetArmGripperMsgs()

            ctrl_mode = getattr(arm_st.arm_status, "ctrl_mode", 0)
            arm_status = getattr(arm_st.arm_status, "arm_status", 0)
            teach_status = getattr(arm_st.arm_status, "teach_status", 0)

            j1 = getattr(joints_msg.joint_state, "joint_1", 0) / 1000.0
            j2 = getattr(joints_msg.joint_state, "joint_2", 0) / 1000.0
            j3 = getattr(joints_msg.joint_state, "joint_3", 0) / 1000.0
            j4 = getattr(joints_msg.joint_state, "joint_4", 0) / 1000.0
            j5 = getattr(joints_msg.joint_state, "joint_5", 0) / 1000.0
            j6 = getattr(joints_msg.joint_state, "joint_6", 0) / 1000.0
            grp = getattr(grp_msg.gripper_state, "grippers_angle", 0) / 1000.0

            mode_name = CONTROL_MODE_NAMES.get(ctrl_mode, f"0x{ctrl_mode:02X}")
            status_name = ARM_STATUS_NAMES.get(arm_status, f"0x{arm_status:02X}")
            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # Detect state transitions (button presses)
            state_changed = (ctrl_mode != prev_mode) or (arm_status != prev_status) or (teach_status != prev_teach)
            if state_changed:
                banner = ""
                if arm_status == 0x0B or teach_status == 1:
                    banner = " >>> [BUTTON ACTION: DRAG TEACH RECORDING STARTED (Solid Green Light)] <<<"
                elif arm_status == 0x0C:
                    banner = " >>> [BUTTON ACTION: TRAJECTORY PLAYBACK STARTED (Flashing Green Light)] <<<"
                elif prev_status in (0x0B, 0x0C) and arm_status == 0x00:
                    banner = " >>> [STATE UPDATE: RETURNED TO NORMAL / IDLE] <<<"

                print(f"\n[{now}] *** STATE CHANGE DETECTED *** Mode: {mode_name} | Status: {status_name}{banner}")
                prev_mode = ctrl_mode
                prev_status = arm_status
                prev_teach = teach_status

            print(
                f"\r[{now}] Mode: {mode_name:<20s} | Status: {status_name:<30s} | "
                f"J1: {j1:6.1f}°, J2: {j2:6.1f}°, J3: {j3:6.1f}°, J4: {j4:6.1f}°, J5: {j5:6.1f}°, J6: {j6:6.1f}° | Grp: {grp:4.1f}mm",
                end="",
                flush=True,
            )

            if csv_writer:
                csv_writer.writerow([now, mode_name, status_name, teach_status, f"[{j1},{j2},{j3},{j4},{j5},{j6}]", grp])

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[INFO] Monitoring session ended.")
    finally:
        piper.DisconnectPort()
        if csv_file:
            csv_file.close()
            print(f"[INFO] Log saved to {log_csv}")


def main():
    parser = argparse.ArgumentParser(description="Piper Live CAN Sniffer & Button Monitor")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel")
    parser.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate")
    parser.add_argument("--log", type=str, default=None, help="Save to CSV")
    parser.add_argument("--mock", action="store_true", help="Simulation mode")
    args = parser.parse_args()

    run_monitor(
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
        log_csv=args.log,
        mock=args.mock,
    )


if __name__ == "__main__":
    main()
