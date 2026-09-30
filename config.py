"""Configuration and Constants for Piper Robot Control.

This module defines CAN bus settings, firmware state constants,
and human-readable status decoders based on the AgileX Piper specification.
"""

from dataclasses import dataclass
from typing import Dict


@dataclass
class CANConfig:
    """CAN Bus Connection Configuration."""
    # Interface types:
    #   'agx_cando': AgileX official USB-to-CAN adapter on Windows (default)
    #   'slcan': Serial-based CAN adapter (e.g. COM3, COM4)
    #   'socketcan': Linux SocketCAN (e.g. can0)
    #   'virtual': Virtual CAN for offline testing/simulation
    interface: str = "agx_cando"
    channel: str = "0"
    bitrate: int = 1000000  # 1 Mbps required by Piper arm
    can_auto_init: bool = False
    judge_flag: bool = False


# Drag Teaching Control Values (MotionCtrl_1 grag_teach_ctrl, CAN ID: 0x150)
class DragTeachCmd:
    NO_OP = 0x00               # 0x00: Disable / No operation
    START_RECORD = 0x01        # 0x01: Start teaching recording (enter drag teach)
    STOP_RECORD = 0x02         # 0x02: Stop teaching recording (exit drag teach)
    EXECUTE_TRAJECTORY = 0x03  # 0x03: Execute taught trajectory (PLAYBACK - replaces double-tap)
    PAUSE = 0x04               # 0x04: Pause execution
    RESUME = 0x05              # 0x05: Continue execution (resume playback)
    TERMINATE = 0x06           # 0x06: Terminate execution
    MOVE_TO_START = 0x07       # 0x07: Move to trajectory start point


DRAG_TEACH_CMD_NAMES = {
    DragTeachCmd.NO_OP: "No Operation",
    DragTeachCmd.START_RECORD: "Start Teaching Recording",
    DragTeachCmd.STOP_RECORD: "Stop Teaching Recording",
    DragTeachCmd.EXECUTE_TRAJECTORY: "Execute Taught Trajectory (Playback)",
    DragTeachCmd.PAUSE: "Pause Trajectory Execution",
    DragTeachCmd.RESUME: "Resume Trajectory Execution",
    DragTeachCmd.TERMINATE: "Terminate Trajectory Execution",
    DragTeachCmd.MOVE_TO_START: "Move to Trajectory Start Point",
}


# Robot Control Mode (GetArmStatus ctrl_mode, CAN ID: 0x2A1)
class ControlMode:
    STANDBY = 0x00
    CAN_COMMAND = 0x01
    TEACH_MODE = 0x02


CONTROL_MODE_NAMES = {
    ControlMode.STANDBY: "Standby Mode",
    ControlMode.CAN_COMMAND: "CAN Command Control Mode",
    ControlMode.TEACH_MODE: "Teach / Demonstration Mode",
}


# Robot Feedback Status (GetArmStatus arm_status, CAN ID: 0x2A1)
class ArmStatus:
    NORMAL = 0x00
    EMERGENCY_STOP = 0x01
    NO_IK_SOLUTION = 0x02
    SINGULARITY = 0x03
    TARGET_ANGLE_EXCEEDED = 0x04
    JOINT_COMM_ERROR = 0x05
    BRAKE_NOT_RELEASED = 0x06
    COLLISION_DETECTED = 0x07
    DRAG_TEACH_OVERSPEED = 0x08
    JOINT_STATUS_ABNORMAL = 0x09
    OTHER_ABNORMALITY = 0x0A
    TEACH_RECORDING = 0x0B     # Robot is currently recording drag-teach trajectory
    TEACH_EXECUTING = 0x0C     # Robot is currently executing/replaying trajectory
    TEACH_PAUSED = 0x0D        # Robot trajectory replay is paused
    MAIN_NTC_OVERTEMP = 0x0E
    RESISTOR_NTC_OVERTEMP = 0x0F


ARM_STATUS_NAMES = {
    ArmStatus.NORMAL: "Normal / Idle",
    ArmStatus.EMERGENCY_STOP: "Emergency Stop Active",
    ArmStatus.NO_IK_SOLUTION: "No Inverse Kinematics Solution",
    ArmStatus.SINGULARITY: "Singularity Detected",
    ArmStatus.TARGET_ANGLE_EXCEEDED: "Target Angle Out of Range",
    ArmStatus.JOINT_COMM_ERROR: "Joint Communication Error",
    ArmStatus.BRAKE_NOT_RELEASED: "Brake Not Released",
    ArmStatus.COLLISION_DETECTED: "Collision Detected",
    ArmStatus.DRAG_TEACH_OVERSPEED: "Drag-Teach Overspeed",
    ArmStatus.JOINT_STATUS_ABNORMAL: "Joint Status Abnormal",
    ArmStatus.OTHER_ABNORMALITY: "Other System Abnormality",
    ArmStatus.TEACH_RECORDING: "Teaching Recording in Progress (Solid Green LED)",
    ArmStatus.TEACH_EXECUTING: "Teaching Playback in Progress (Flashing Green LED)",
    ArmStatus.TEACH_PAUSED: "Teaching Playback Paused",
    ArmStatus.MAIN_NTC_OVERTEMP: "Main Controller Over-Temperature",
    ArmStatus.RESISTOR_NTC_OVERTEMP: "Discharge Resistor Over-Temperature",
}


# CAN Arbitration IDs for Piper Arm
class PiperCANID:
    MOTION_CTRL_1 = 0x150
    MOTION_CTRL_2 = 0x151
    JOINT_CTRL = 0x155
    GRIPPER_CTRL = 0x159
    ARM_STATUS_FEEDBACK = 0x2A1
    END_POSE_FEEDBACK = 0x2A2
    JOINT_1_2_FEEDBACK = 0x251
    JOINT_3_4_FEEDBACK = 0x252
    JOINT_5_6_FEEDBACK = 0x253
    GRIPPER_FEEDBACK = 0x254
    MOTOR_ENABLE_DISABLE = 0x471
    FIRMWARE_QUERY = 0x473
