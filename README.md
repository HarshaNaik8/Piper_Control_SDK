# AgileX Piper Robotic Arm - Camera Sync Controller

A software suite to control the **AgileX Piper 6-DOF Robotic Arm** from a Windows laptop over a USB-to-CAN adapter using the official AgileX Python SDK.

## ⚠️ Important Notice: Project Status

**This project is currently under active development and is not 100% verified across all features.**
* **What is READY & TESTED:** The core implementation—the **Trajectory Replay button** in the GUI and via the CLI. It successfully and safely triggers the robot without dropping torque.
* **What is UNVERIFIED:** Additional safety features (Emergency Stop / Dead Weight, Hold Position) and Gripper Enable/Disable controls have been implemented in the code based on CAN protocols but **have not yet been fully physically tested or verified** to guarantee they won't interfere with the firmware state. Use these secondary buttons with caution.

---

## 🎯 Goal

Replace the manual, repetitive physical drag-teach button operation (double-tapping to replay a recorded motion) with an automated software trigger over CAN bus (`grag_teach_ctrl = 0x03` on CAN ID `0x150`).

This enables **Zero-Latency Multi-Camera Dataset Collection**: your laptop triggers the motion, perfectly synchronizes with 3x Samsung mobile camera recordings (via a mentor script), waits for completion, and loops across episodes without any human needing to touch the robot arm during the actual dataset capture.

---

## 🔄 The Replay Button Workflow (Crucial)

**How it works:** The GUI Replay button does *not* work completely blindly out of the box. **You must "prime" the action physically once.**

1. Single-click the physical green button on the arm to enter drag-teach mode.
2. Move the arm through your desired task.
3. Single-click the physical button again to finish recording.
4. **The "Prime" Step:** Double-click the physical button to make the robot mimic the action once. 

**Why is this required?**
We intentionally left the workflow this way! It acts as a mandatory verification step. Before you hand over control to the automated GUI and start recording 100+ episodes of dataset videos, you *must* physically watch the robot perform the task once to ensure it is correct and safe. Once verified, you can step away and use the GUI Replay button (or the `piper_sync_trigger.py` script) to repeat the exact same task indefinitely. This is a safety feature, not a bug.

---

## 🏗️ Architecture

```text
Windows 11 Laptop
  ├── Python 3.13 Virtual Environment (venv)
  │     ├── piper_sdk (0.6.2)
  │     ├── python-can (4.6.1)
  │     └── python-can-agx-cando (AgileX native cando.dll backend)
  │
  └── USB Port
        │
   USB-CAN Adapter (AgileX CANdo)
        │ CAN-H / CAN-L (1 Mbps)
   Piper Aviation Interface (Pins 6 & 7)
        │
   Piper Integrated Controller (Firmware)
        │
   6-DOF Motors & Gripper
```

---

## 🚀 Quick Start Guide

### 1. Activate Environment
In PowerShell:
```powershell
.\venv\Scripts\Activate.ps1
```

### 2. Physical Hardware Setup
1. **Power**: Plug the Piper 24V DC power adapter into the aviation power port and switch it on.
2. **CAN Wiring**:
   - Aviation Pin 6: CAN-H (Yellow / High)
   - Aviation Pin 7: CAN-L (Blue / Low)
3. **USB**: Connect the USB-CAN adapter to your laptop's USB port.

### 3. Launch the Simple GUI
Launch the fully stripped-down, safe desktop dashboard:
```powershell
python piper_simple_gui.py
```
*Features:*
- Big, unambiguous 'REPLAY TRAJECTORY' button.
- Clean Gripper controls: Enable (Stiff), Disable (Loose). *(Pending verification)*
- Safety Controls: Hold Position vs Dead Weight. *(Pending verification)*
- Zero dangerous pause/resume toggles that conflict with the arm's firmware.

---

## 📊 Dataset Collection Integration

Use `piper_sync_trigger.py` to seamlessly integrate with external camera recording scripts:

```python
from piper_sync_trigger import PiperSyncTrigger

robot = PiperSyncTrigger(interface="agx_cando", channel="0")
if not robot.connect():
    raise RuntimeError("Failed to connect.")

for episode in range(10):
    print(f"Recording episode {episode}...")
    
    # 1. Start your Samsung camera recordings here
    # ...
    
    # 2. Trigger the taught trajectory simultaneously (Zero ms latency)
    # This blocks until the robot finishes moving.
    robot.replay_and_wait(timeout=60.0)
    
    # 3. Stop your cameras and save episode data
    # ...

robot.disconnect()
```

---

## 📁 Repository Structure

| File | Description |
|---|---|
| `config.py` | CAN interface settings, protocol IDs, and status code decoders |
| `piper_playback.py` | Core safe CAN playback controller (No torque drops) |
| `piper_simple_gui.py` | Clean Tkinter GUI focused on Replay, Hold, and Dead Weight |
| `piper_sync_trigger.py` | CLI and API wrapper for zero-latency video script integration |
| `dataset_trigger_demo.py` | Automated multi-episode trajectory playback demo |
| `hardware_check.py` | Level 1 & 2: USB device and CAN bus detection |
| `can_monitor.py` | Level 3: Passive CAN frame sniffer & CSV logger |
| `requirements.txt` | Python package dependencies |
