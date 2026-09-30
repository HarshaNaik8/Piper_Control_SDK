"""Receive-Only CAN Monitor / Sniffer Tool (Testing Ladder Level 3).

Allows observing live CAN traffic without sending any motion commands.
Use this to capture what the Piper broadcasts when you physically
press or double-tap the drag-teach button between J5 and J6.
"""

import sys
import time
import csv
import argparse
from datetime import datetime
from typing import Optional

try:
    import can
except ImportError:
    print("[ERROR] python-can is not installed.")
    sys.exit(1)

from config import (
    CANConfig,
    PiperCANID,
    CONTROL_MODE_NAMES,
    ARM_STATUS_NAMES,
    DRAG_TEACH_CMD_NAMES,
)


def decode_arm_status(data: bytes) -> str:
    """Decode 0x2A1 Arm Status Feedback message."""
    if len(data) < 7:
        return f"Raw: {data.hex()}"
    ctrl_mode = data[0]
    arm_status = data[1]
    mode_feed = data[2]
    teach_status = data[3]
    motion_status = data[4]
    trajectory_num = int.from_bytes(data[5:7], byteorder="little")

    ctrl_str = CONTROL_MODE_NAMES.get(ctrl_mode, f"0x{ctrl_mode:02X}")
    status_str = ARM_STATUS_NAMES.get(arm_status, f"0x{arm_status:02X}")
    return f"Mode: [{ctrl_str}] | Status: [{status_str}] | TrajPt: {trajectory_num}"


def decode_joints(id: int, data: bytes) -> str:
    """Decode joint feedback frames (0x251, 0x252, 0x253)."""
    if len(data) < 8:
        return f"Raw: {data.hex()}"
    # Each joint angle is int32 (little endian) in 0.001 deg or 0.001 rad
    j_a = int.from_bytes(data[0:4], byteorder="little", signed=True) / 1000.0
    j_b = int.from_bytes(data[4:8], byteorder="little", signed=True) / 1000.0
    if id == PiperCANID.JOINT_1_2_FEEDBACK:
        return f"J1: {j_a:7.2f}°, J2: {j_b:7.2f}°"
    elif id == PiperCANID.JOINT_3_4_FEEDBACK:
        return f"J3: {j_a:7.2f}°, J4: {j_b:7.2f}°"
    elif id == PiperCANID.JOINT_5_6_FEEDBACK:
        return f"J5: {j_a:7.2f}°, J6: {j_b:7.2f}°"
    return f"Raw: {data.hex()}"


def decode_gripper(data: bytes) -> str:
    """Decode 0x254 Gripper Feedback message."""
    if len(data) < 6:
        return f"Raw: {data.hex()}"
    stroke = int.from_bytes(data[0:4], byteorder="little", signed=True) / 1000.0
    effort = int.from_bytes(data[4:6], byteorder="little", signed=True) / 1000.0
    return f"Stroke: {stroke:6.2f} mm | Effort: {effort:5.2f} N"


def decode_motion_ctrl_1(data: bytes) -> str:
    """Decode 0x150 Motion Control 1 message."""
    if len(data) < 3:
        return f"Raw: {data.hex()}"
    estop = data[0]
    track = data[1]
    teach = data[2]
    teach_str = DRAG_TEACH_CMD_NAMES.get(teach, f"0x{teach:02X}")
    return f"E-Stop: {estop}, TrackCtrl: {track}, DragTeachCmd: [{teach_str}]"


def start_monitor(
    interface: str = "agx_cando",
    channel: str = "0",
    bitrate: int = 1000000,
    log_csv: Optional[str] = None,
    filter_status_only: bool = False,
    mock: bool = False,
) -> None:
    """Start passive CAN sniffer loop."""
    print("=" * 75)
    print(" PASSIVE CAN OBSERVATION MONITOR (LEVEL 3)")
    print(f" Interface : {interface} (Channel: {channel}, Bitrate: {bitrate})")
    print(f" Mode      : Receive-Only (No motion commands will be sent)")
    if log_csv:
        print(f" Logging to: {log_csv}")
    print(" Press Ctrl+C at any time to stop monitoring.")
    print("=" * 75)

    csv_file = None
    csv_writer = None
    if log_csv:
        csv_file = open(log_csv, mode="w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Timestamp", "CAN_ID_Hex", "CAN_ID_Dec", "DLC", "Data_Hex", "Decoded_Info"])

    if mock:
        print("\n[INFO] Running in SIMULATION / MOCK mode (generating simulated Piper status frames)...")
        import random
        try:
            step = 0
            while True:
                time.sleep(0.2)
                step += 1
                # simulate arm status (0x2A1) transitioning to playback every 10 steps
                status = 0x0C if (10 <= step % 25 <= 20) else 0x00
                data = bytes([0x01, status, 0x01, 0x00, 0x00, step % 255, 0x00, 0x00])
                decoded = decode_arm_status(data)
                now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{now_str}] ID: 0x2A1 | DLC: 8 | DATA: {data.hex().upper()} | {decoded}")
                if csv_writer:
                    csv_writer.writerow([now_str, "0x2A1", 0x2A1, 8, data.hex(), decoded])
        except KeyboardInterrupt:
            print("\n[INFO] Simulated monitor stopped.")
            if csv_file:
                csv_file.close()
        return

    bus = None
    try:
        bus = can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            receive_own_messages=False,
        )
        print("\n[+] CAN Bus successfully opened in listen mode. Waiting for frames...\n")

        msg_count = 0
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue

            msg_count += 1
            can_id = msg.arbitration_id
            data = bytes(msg.data)
            hex_id = f"0x{can_id:03X}"
            data_hex = data.hex().upper()
            now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            decoded = ""
            if can_id == PiperCANID.ARM_STATUS_FEEDBACK:
                decoded = decode_arm_status(data)
            elif can_id in (PiperCANID.JOINT_1_2_FEEDBACK, PiperCANID.JOINT_3_4_FEEDBACK, PiperCANID.JOINT_5_6_FEEDBACK):
                decoded = decode_joints(can_id, data)
            elif can_id == PiperCANID.GRIPPER_FEEDBACK:
                decoded = decode_gripper(data)
            elif can_id == PiperCANID.MOTION_CTRL_1:
                decoded = f"*** COMMAND DETECTED *** {decode_motion_ctrl_1(data)}"
            else:
                decoded = f"Raw Frame: {data_hex}"

            if filter_status_only and can_id != PiperCANID.ARM_STATUS_FEEDBACK:
                continue

            print(f"[{now_str}] ID: {hex_id} | DLC: {msg.dlc} | DATA: {data_hex:16s} | {decoded}")

            if csv_writer:
                csv_writer.writerow([now_str, hex_id, can_id, msg.dlc, data_hex, decoded])
                csv_file.flush()

    except can.CanInitializationError as e:
        print(f"\n[ERROR] CAN bus initialization failed: {e}")
        print("Ensure the USB-CAN adapter is plugged in and recognized.")
    except KeyboardInterrupt:
        print(f"\n[INFO] Monitoring terminated by user. Captured {msg_count} packets.")
    finally:
        if bus is not None:
            bus.shutdown()
            print("[INFO] CAN bus safely closed.")
        if csv_file is not None:
            csv_file.close()
            print(f"[INFO] Frame log written to: {log_csv}")


def main():
    parser = argparse.ArgumentParser(description="Piper Passive CAN Sniffer / Monitor")
    parser.add_argument("--interface", type=str, default="agx_cando", help="CAN interface (agx_cando, slcan, socketcan)")
    parser.add_argument("--channel", type=str, default="0", help="CAN channel index or port (0, COM3, can0)")
    parser.add_argument("--bitrate", type=int, default=1000000, help="Bitrate (default: 1000000)")
    parser.add_argument("--log", type=str, default=None, help="Save captured packets to CSV file")
    parser.add_argument("--status-only", action="store_true", help="Display only 0x2A1 status feedback")
    parser.add_argument("--mock", action="store_true", help="Run with simulated frames for offline testing")
    args = parser.parse_args()

    start_monitor(
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
        log_csv=args.log,
        filter_status_only=args.status_only,
        mock=args.mock,
    )


if __name__ == "__main__":
    main()
