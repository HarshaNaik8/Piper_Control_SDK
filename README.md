# AgileX Piper Robotic Arm Laptop Controller

A software suite to control the **AgileX Piper 6-DOF Robotic Arm** from a Windows laptop over a USB-to-CAN adapter using the official AgileX Python SDK.

---

## 🎯 Goal

Replace the physical drag-teach button operation (double-tapping the button between J5 and J6 to replay a recorded motion) with an automated software trigger over CAN bus (`grag_teach_ctrl = 0x03` on CAN ID `0x150`).

This enables **100% hands-free dataset collection**: your laptop triggers the motion, waits for completion, saves sensor/camera data, and loops across episodes without any human needing to touch the robot arm.

```
+-------------------------------------------------------------+
| Physical Workflow (Old):                                    |
|   Hand move arm -> Single-tap -> Double-tap button manually |
+-------------------------------------------------------------+
                               |
                               v
+-------------------------------------------------------------+
| Laptop Control Workflow (New):                              |
|   Laptop -> Python SDK -> python-can -> USB-CAN -> Piper     |
|   Command: MotionCtrl_1(grag_teach_ctrl=0x03)                |
+-------------------------------------------------------------+
```

---

## 🏗️ Architecture

```
Windows 11 Laptop
  │
  ├── Python 3.13 Virtual Environment (venv)
  │     ├── piper_sdk (0.6.2)
  │     ├── python-can (4.6.1)
  │     └── python-can-agx-cando (AgileX native cando.dll backend)
  │
  └── USB Port
        │
        ▼
   USB-CAN Adapter (AgileX CANdo)
        │
        ▼ CAN-H / CAN-L (1 Mbps)
   Piper Aviation Interface (Pins 6 & 7)
        │
        ▼
   Piper Integrated Controller (Firmware)
        │
        ▼
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

---

## 🪜 Testing Ladder (Step-by-Step Execution)

Follow the roadmap established in `Piper_control_planning.pdf` from safest to fully active:

### Level 1 & 2: Hardware & CAN Detection
Scan USB devices and confirm CAN initialization (transmits zero motion commands):
```powershell
python hardware_check.py
```

### Level 3: Passive CAN Sniffer / Monitor (Listen Only)
Passively listen to the robot's broadcast frames without moving anything. Use this to observe what the robot sends when you press the button:
```powershell
python can_monitor.py
```
*Tip: To save frames to CSV, run: `python can_monitor.py --log button_traffic.csv`*

### Level 4 & 5: Live State & Telemetry Monitor
Read live joint angles (J1-J6), gripper stroke, and operational status:
```powershell
python piper_state.py
```

### Level 6: Trajectory Playback (Double-Tap Software Replacement)
Trigger the recorded trajectory directly from the command line:
```powershell
python piper_cli.py play
```
*Additional CLI commands:*
- Pause: `python piper_cli.py pause`
- Resume: `python piper_cli.py resume`
- Stop: `python piper_cli.py stop`
- Interactive Console: `python piper_cli.py interactive`

### Level 7: Interactive Graphical User Interface (GUI)
Launch the full desktop dashboard:
```powershell
python piper_gui.py
```
*Features:*
- Connect/Disconnect toggle
- Live 6-DOF joint angles & gripper stroke
- Real-time status display (Mode, Arm Status, CAN FPS)
- Play, Pause, Resume, Stop, Move to Start buttons
- Drag-Teach record triggers
- Motor Enable/Disable and Emergency Stop
- Real-time event log
- **Offline Simulation Mode** checkbox to test UI without hardware attached

---

## 🤖 Dataset Collection Integration

Use `dataset_trigger_demo.py` in your automated data collection pipelines:

```python
from piper_controller import PiperRobotController

robot = PiperRobotController(interface="agx_cando", channel="0")
robot.connect()
robot.enable_arm()

for episode in range(10):
    print(f"Recording episode {episode}...")
    
    # 1. Trigger the taught trajectory (no button press needed)
    robot.execute_taught_trajectory()
    
    # 2. Wait until motion finishes while your cameras record
    robot.wait_for_trajectory_completion(timeout=60.0)
    
    # 3. Save episode data and proceed to next

robot.disconnect()
```

Run demo:
```powershell
# With real robot:
python dataset_trigger_demo.py --episodes 5

# Offline simulation test:
python dataset_trigger_demo.py --episodes 3 --mock
```

---

## 📁 Repository Structure

| File | Description |
|---|---|
| `config.py` | CAN interface settings, protocol IDs, and status code decoders |
| `hardware_check.py` | Level 1 & 2: USB device and CAN bus detection |
| `can_monitor.py` | Level 3: Passive CAN frame sniffer & CSV logger |
| `piper_controller.py` | Core API class (`PiperRobotController`) handling SDK calls |
| `piper_state.py` | Level 4 & 5: Live joint angle & status terminal reader |
| `piper_cli.py` | Command-line tool with interactive text control menu |
| `piper_gui.py` | Level 7: Full Tkinter desktop dashboard |
| `dataset_trigger_demo.py` | Automated multi-episode trajectory playback runner |
| `requirements.txt` | Python package requirements |
