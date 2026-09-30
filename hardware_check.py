"""Hardware Detection and CAN Interface Diagnostic Tool (Testing Ladder Level 1 & 2).

This script verifies:
  Level 1: Hardware Detection (Checks USB devices, COM ports, and AgileX CANdo adapters).
  Level 2: CAN Communication Check (Initializes CAN interface at 1Mbps safely without moving the robot).
"""

import sys
import time
import platform
import subprocess
from typing import List, Dict, Any

try:
    import can
except ImportError:
    print("[ERROR] python-can is not installed. Run: pip install python-can")
    sys.exit(1)

# Check for agx_cando backend
AGX_CANDO_AVAILABLE = False
try:
    from agx_cando.bus import AgxCandoBus
    AGX_CANDO_AVAILABLE = True
except ImportError:
    AGX_CANDO_AVAILABLE = False

try:
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def print_banner(text: str) -> None:
    width = 70
    print("\n" + "=" * width)
    print(f" {text}")
    print("=" * width)


def scan_cando_devices() -> List[Dict[str, Any]]:
    """Scan for AgileX CANdo USB-CAN adapters."""
    if not AGX_CANDO_AVAILABLE:
        return []
    try:
        devices = AgxCandoBus._detect_available_configs()
        return devices
    except Exception as e:
        print(f"  [!] Exception during AgxCando scan: {e}")
        return []


def scan_serial_ports() -> List[Dict[str, str]]:
    """Scan for serial / COM ports (e.g. SLCAN devices)."""
    if not SERIAL_AVAILABLE:
        return []
    results = []
    try:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            results.append({
                "port": p.device,
                "description": p.description,
                "hwid": p.hwid,
            })
    except Exception as e:
        print(f"  [!] Exception scanning serial ports: {e}")
    return results


def check_level_1_hardware() -> Dict[str, Any]:
    """Execute Level 1: Detect hardware presence."""
    print_banner("LEVEL 1: Hardware Detection (No Robot Movement)")
    print(f"Operating System : {platform.system()} {platform.release()} ({platform.architecture()[0]})")
    print(f"Python Version   : {sys.version.split()[0]}")
    print(f"python-can       : {can.__version__}")
    print(f"agx_cando driver : {'Installed' if AGX_CANDO_AVAILABLE else 'NOT installed'}")

    cando_devices = scan_cando_devices()
    print(f"\n[1] Scanning for AgileX CANdo USB Adapters:")
    if cando_devices:
        for idx, dev in enumerate(cando_devices):
            print(f"  --> Found CANdo device #{idx}: interface={dev.get('interface')}, channel={dev.get('channel')}")
    else:
        print("  --> No AgileX CANdo USB adapter currently detected.")

    print(f"\n[2] Scanning for Serial / COM Ports:")
    serial_devices = scan_serial_ports()
    if serial_devices:
        for s in serial_devices:
            print(f"  --> Port: {s['port']} | {s['description']}")
    else:
        print("  --> No COM ports found.")

    is_detected = (len(cando_devices) > 0) or (len(serial_devices) > 0)
    return {
        "cando_devices": cando_devices,
        "serial_devices": serial_devices,
        "hardware_present": is_detected,
    }


def check_level_2_can_init(interface: str = "agx_cando", channel: str = "0", bitrate: int = 1000000) -> bool:
    """Execute Level 2: Establish CAN communication without sending commands."""
    print_banner("LEVEL 2: Establish CAN Bus Communication")
    print(f"Attempting test connection to: interface='{interface}', channel='{channel}', bitrate={bitrate}...")

    bus = None
    try:
        bus = can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            receive_own_messages=False,
        )
        print("  [SUCCESS] CAN bus opened successfully!")
        print(f"  Bus Channel Info: {getattr(bus, 'channel_info', 'N/A')}")
        print("  Verification: Zero packets transmitted. Passive test passed.")
        return True
    except can.CanInitializationError as cie:
        print(f"  [ERROR] CAN Initialization failed: {cie}")
        return False
    except Exception as e:
        print(f"  [ERROR] Failed to connect: {e}")
        return False
    finally:
        if bus is not None:
            try:
                bus.shutdown()
                print("  [INFO] Test CAN bus session closed cleanly.")
            except Exception as e:
                print(f"  [!] Warning on shutdown: {e}")


def main():
    hw_info = check_level_1_hardware()

    if hw_info["cando_devices"]:
        first_channel = str(hw_info["cando_devices"][0].get("channel", "0"))
        print(f"\n[+] AgileX CAN adapter detected on channel '{first_channel}'. Proceeding to Level 2...")
        success = check_level_2_can_init(interface="agx_cando", channel=first_channel, bitrate=1000000)
        if success:
            print("\n>>> Level 1 & 2 PASSED. Hardware and CAN layers are ready for Level 3 (CAN Sniffer) or Level 4.")
            return 0
        else:
            print("\n>>> Level 2 FAILED. Check USB cable and driver permissions.")
            return 1
    else:
        print("\n" + "-" * 70)
        print("[NOTICE] USB-CAN adapter is NOT currently plugged in or powered on.")
        print("To complete physical connection:")
        print("  1. Plug the Piper USB-CAN adapter into your laptop USB port.")
        print("  2. Ensure the Piper 24V power supply is switched ON.")
        print("  3. Ensure CAN-H and CAN-L wires are properly connected.")
        print("  4. Re-run: python hardware_check.py")
        print("-" * 70)
        return 0


if __name__ == "__main__":
    sys.exit(main())
