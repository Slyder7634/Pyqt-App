#!/usr/bin/env python3
"""
NextUp Robot Control — PyQt5 Point Planning page with ROS 2 integration.
STRICT RAW DATA MODE: Shows exact values from topic/YAML with NO unit conversion.
"""

import os
import re
import sys
import importlib.util
import threading
import time
import math
from copy import deepcopy
from datetime import datetime
import json

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool
from std_srvs.srv import SetBool, Trigger

# ethercat_msgs is a custom package. If it isn't on the path the app still
# runs; mode switching then reports the missing dependency instead of
# silently doing nothing.
try:
    from ethercat_msgs.srv import SetSdo
except ImportError:
    SetSdo = None

# nextup_joint_interfaces carries the status messages. Imported defensively so
# the UI still starts (with dead LEDs) if the package is not on the path.
try:
    from nextup_joint_interfaces.msg import (
        NextupDriverStatus, NextupDigitalInputs, NextupJointState,
        NextupEmergencyTrigger,
    )
except ImportError:
    NextupDriverStatus = None
    NextupDigitalInputs = None
    NextupJointState = None
    NextupEmergencyTrigger = None
from PyQt5 import QtCore, QtWidgets, uic


HERE = os.path.dirname(os.path.abspath(__file__))
UI_FILE = os.path.join(HERE, "main_window.ui")
QSS_FILE = os.path.join(HERE, "style.qss")

POINTS_FILE = "/home/nextup/NextupRobot/src/active_project_configs/planning_data/points.yaml"

# Overspeed thresholds written over EtherCAT when the mode changes.
# Mirrors nextupWeb/config/overspeed_threshold_limits.yaml — if you edit that
# file, edit this too (or point OVERSPEED_YAML at it and load from disk).
OVERSPEED_YAML = "/home/nextup/NextupRobot/src/nextupWeb/config/overspeed_threshold_limits.yaml"

OVERSPEED_FALLBACK = {
    "index": 0x200A,
    "subindex": 0x09,
    "type": "uint16",
    "auto":     {f"joint{i}": 6000 for i in range(1, 7)},
    "safeauto": {f"joint{i}": 1000 for i in range(1, 7)},
    "jog":      {f"joint{i}": 600 for i in range(1, 7)},
}

# UI mode -> the value published on /change_mode.
# Note auto and safeauto share drive mode 8; they differ ONLY in the overspeed
# thresholds, so switching between them writes SDOs and publishes nothing.
MODE_DRIVE = {"auto": "8", "safeauto": "8", "jog": "9"}

JOINTS = [f"joint{i}" for i in range(1, 7)]

CALIBRATION_FILE = "/home/nextup/NextupRobot/src/active_project_configs/planning_data/calibration.yaml"
SERVO_FRAME_FILE = "/home/nextup/NextupRobot/src/nextupWeb/user_config/frame.json"

# A calibration frame is defined by exactly these four slots.
POINT_SLOTS = ["point1", "point2", "point3", "point4"]

# "end" means "no transform" — ui_command_node publishes the twist directly
# instead of rotating it through TF. Every other entry must be a real TF frame.
DEFAULT_FRAME = "end"

# Shown under "SPECIAL FRAMES" in the servo frame dropdown: (label, frame id).
# "end" means no transform — ui_command_node publishes the twist directly.
# ws_sim viewer, launched as a child process by the NexSim button.
WS_SIM_SCRIPT = os.path.join(HERE, "ws_sim.py")
NEXTUP_HMI_DIR = os.path.expanduser("~/nextup_hmi")

SPECIAL_FRAMES = [
    ("END", "end"),
    ("BASE", "base_link"),
    ("EXTERNAL", "external"),
]

SDO_TIMEOUT_S = 0.3

JOINT_AXES = ["J1", "J2", "J3", "J4", "J5", "J6"]
CART_AXES = ["X", "Y", "Z", "R", "P", "W"]


# Cartesian axis letters expected by ui_command_node (1-based index).
CART_AXIS_CHARS = {1: "x", 2: "y", 3: "z", 4: "r", 5: "p", 6: "w"}

# How often to repeat the command while a jog button is held.
# The velocity ramp lives in ui_command_node.cpp, which accelerates only when
# it receives the SAME string twice in a row. A single message therefore
# publishes zero velocity and the robot does not move; motion requires the
# repeat. The stop string is sent exactly once because the node runs its own
# blocking deceleration loop on receiving it.
JOG_REPEAT_MS = 30


def restyle(widget):
    """Re-apply QSS after a dynamic property changes."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)


class ROSNode(Node):
    """ROS 2 node for the UI."""
    
    def __init__(self):
        super().__init__('ui_controller')
        # /ui_commands is std_msgs/String, NOT Float64MultiArray.
        # ui_command_node.cpp subscribes as String; a mismatched type means
        # the subscription never connects and nothing arrives.
        self.ui_cmd_pub = self.create_publisher(String, '/ui_commands', 10)
        self.change_mode_pub = self.create_publisher(String, '/change_mode', 10)

        # Service clients are created unconditionally; availability is checked
        # at call time so a missing node produces an error message rather than
        # crashing the UI at startup.
        self.planner_cli = self.create_client(SetBool, '/change_planning_pipeline')
        self.start_servo_cli = self.create_client(Trigger, '/servo_node/start_servo')

        # Motion-type flags consumed by moveit_go_to_pose. It requires exactly
        # one of these before it will act on a get_last_pose@ command.
        self.joint_motion_pub = self.create_publisher(Bool, '/joint_motion', 10)
        self.cartesian_motion_pub = self.create_publisher(Bool, '/cartesian_motion', 10)

        # Workspace calibration topics.
        self.start_calib_pub = self.create_publisher(String, '/start_calibration', 10)
        self.frame_origin_pub = self.create_publisher(String, '/go_to_frame_origin', 10)
        self.finish_calib_pub = self.create_publisher(Bool, '/finish_ws_calibration', 10)
        self.go_to_tf_point_pub = self.create_publisher(String, '/go_to_tf_point', 10)

        self.frame_mode_pub = self.create_publisher(String, '/frame_mode', 10)
        self.set_sdo_cli = None
        if SetSdo is not None:
            self.set_sdo_cli = self.create_client(SetSdo, '/ethercat_manager/set_sdo')

        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_values',
            self.joint_callback,
            10
        )
        self.cart_sub = self.create_subscription(
            Float64MultiArray,
            '/cartesian_values',
            self.cart_callback,
            10
        )
        self.joint_data = None
        self.cart_data = None

        # ── status / action plumbing ─────────────────────────────────────
        self.reset_fault_pub = self.create_publisher(Bool, '/reset_fault', 10)
        self.traj_event_pub = self.create_publisher(
            String, '/trajectory_execution_event', 10)
        self.emergency_pub = None
        if NextupEmergencyTrigger is not None:
            self.emergency_pub = self.create_publisher(
                NextupEmergencyTrigger,
                '/nextup_emergency_trigger_controller/commands', 10)

        # Status, all six-element bool lists in joint1..joint6 order.
        self._status_lock = threading.Lock()
        self.op_status = [False] * 6
        self.fault_status = [False] * 6
        self.homing_status = [False] * 6
        self.emergency_status = [False] * 6
        self.drive_mode = None

        if NextupDriverStatus is not None:
            self.create_subscription(
                NextupDriverStatus, '/nextup_driver_status',
                self._driver_status_callback, 10)
        if NextupDigitalInputs is not None:
            self.create_subscription(
                NextupDigitalInputs, '/nextup_digital_inputs',
                self._digital_inputs_callback, 10)
        if NextupJointState is not None:
            self.create_subscription(
                NextupJointState, '/nextup_joint_states',
                self._nextup_joint_state_callback, 10)

    # ── status callbacks ─────────────────────────────────────────────────
    def _driver_status_callback(self, msg):
        """Operational and fault LEDs.

        The message carries a name[] array, and the driver order is not
        guaranteed, so every field is indexed by joint name rather than by
        position. A joint missing from the message reads as false.
        """
        op = [False] * 6
        fault = [False] * 6
        names = list(msg.name)
        for i, joint in enumerate(JOINTS):
            if joint in names:
                idx = names.index(joint)
                if idx < len(msg.op_status):
                    op[i] = bool(msg.op_status[idx])
                if idx < len(msg.fault):
                    fault[i] = bool(msg.fault[idx])
        with self._status_lock:
            self.op_status = op
            self.fault_status = fault

    def _digital_inputs_callback(self, msg):
        """Emergency LEDs come from digital input 5, one per drive."""
        di5 = list(getattr(msg, "di5", []) or [])
        with self._status_lock:
            self.emergency_status = [
                bool(di5[i]) if i < len(di5) else False for i in range(6)]

    def _nextup_joint_state_callback(self, msg):
        """Homing LEDs and the current drive mode.

        A joint counts as homed when its position rounds to zero at three
        decimal places — the same test the web UI uses, so both agree.
        """
        names = list(msg.name)
        homed = [False] * 6
        for i, joint in enumerate(JOINTS):
            if joint in names:
                idx = names.index(joint)
                if idx < len(msg.position):
                    homed[i] = f"{abs(msg.position[idx]):.3f}" == "0.000"

        mode = None
        modes = getattr(msg, "modeofoperation", None)
        if modes is not None and len(modes) > 0:
            mode = int(modes[0])

        with self._status_lock:
            self.homing_status = homed
            if mode is not None:
                self.drive_mode = mode

    def get_status(self):
        """Snapshot of all status rows, for the GUI thread to read."""
        with self._status_lock:
            return {
                "op": list(self.op_status),
                "fault": list(self.fault_status),
                "homing": list(self.homing_status),
                "emergency": list(self.emergency_status),
                "drive_mode": self.drive_mode,
            }

    # ── actions ──────────────────────────────────────────────────────────
    def trigger_emergency(self):
        if self.emergency_pub is None:
            return False
        msg = NextupEmergencyTrigger()
        msg.emergencytrigger = True
        self.emergency_pub.publish(msg)
        return True

    def publish_reset_fault(self):
        msg = Bool()
        msg.data = True
        self.reset_fault_pub.publish(msg)

    def stop_moveit(self):
        """Abort the running trajectory — the native form of stopMoveit()."""
        msg = String()
        msg.data = "stop"
        self.traj_event_pub.publish(msg)

    def publish_ui_command(self, command):
        msg = String()
        msg.data = command
        self.ui_cmd_pub.publish(msg)

    def wait_for_drive_mode(self, target, timeout_s):
        """Block until /nextup_joint_states reports the target drive mode.

        Returns (confirmed, seconds_waited). Already being at the target
        returns immediately, which the HOME sequence relies on to tell a real
        transition apart from a no-op.
        """
        start = time.monotonic()
        deadline = start + timeout_s
        while time.monotonic() < deadline:
            with self._status_lock:
                if self.drive_mode == target:
                    return True, time.monotonic() - start
            time.sleep(0.02)
        return False, time.monotonic() - start

    def joint_callback(self, msg):
        self.joint_data = msg.data
        
    def cart_callback(self, msg):
        self.cart_data = msg.data
        
    def send(self, text):
        """Publish a raw 3-character command string to /ui_commands."""
        msg = String()
        msg.data = text
        self.ui_cmd_pub.publish(msg)

    @staticmethod
    def build_command(mode, axis_index, sign):
        """Build the command ui_command_node parses.

            data[0]  '+' | '-' | '0'
            data[1]  'j' | 'c'
            data[2]  '1'..'6'  (joint)  or  x,y,z,r,p,w  (cartesian)

        axis_index is 1-based.
        """
        d = "+" if sign > 0 else "-"
        if mode == "joint":
            return f"{d}j{axis_index}"
        return f"{d}c{CART_AXIS_CHARS[axis_index]}"

    # ── planner ──────────────────────────────────────────────────────────
    def set_planner(self, use_ompl, timeout_s=5.0):
        """Call /change_planning_pipeline. True = OMPL, False = PILZ.

        Returns (ok, message). Blocking, so call it from a worker thread and
        never from the GUI thread.
        """
        if not self.planner_cli.wait_for_service(timeout_sec=1.0):
            return False, "/change_planning_pipeline not available"

        req = SetBool.Request()
        req.data = bool(use_ompl)
        future = self.planner_cli.call_async(req)

        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return False, "change_planning_pipeline timed out"
            time.sleep(0.01)

        resp = future.result()
        if resp is None:
            return False, "no response"
        return bool(resp.success), resp.message or ""

    # ── overspeed SDO ────────────────────────────────────────────────────
    def write_overspeed(self, slave_position, index, subindex, dtype, value,
                        timeout_s=SDO_TIMEOUT_S):
        """One EtherCAT SDO write. Returns (ok, message)."""
        if self.set_sdo_cli is None:
            return False, "ethercat_msgs not installed"
        if not self.set_sdo_cli.wait_for_service(timeout_sec=0.5):
            return False, "/ethercat_manager/set_sdo not available"

        req = SetSdo.Request()
        req.master_id = 0
        req.slave_position = int(slave_position)
        req.sdo_index = int(index)
        req.sdo_subindex = int(subindex)
        req.sdo_data_type = str(dtype)
        req.sdo_value = str(value)

        future = self.set_sdo_cli.call_async(req)
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return False, f"SetSdo timed out (slave {slave_position})"
            time.sleep(0.005)

        resp = future.result()
        if resp is None:
            return False, "no response"
        return bool(resp.success), getattr(resp, "sdo_return_message", "") or ""

    def publish_change_mode(self, drive_mode):
        msg = String()
        msg.data = str(drive_mode)
        self.change_mode_pub.publish(msg)

    # ── servo ────────────────────────────────────────────────────────────
    def start_servo(self, timeout_s=5.0):
        """Call /servo_node/start_servo. Blocking — use a worker thread."""
        if not self.start_servo_cli.wait_for_service(timeout_sec=1.0):
            return False, "/servo_node/start_servo not available"

        future = self.start_servo_cli.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return False, "start_servo timed out"
            time.sleep(0.01)

        resp = future.result()
        if resp is None:
            return False, "no response"
        return bool(resp.success), resp.message or "Servo started"

    # ── frame ────────────────────────────────────────────────────────────
    def publish_frame_mode(self, frame):
        msg = String()
        msg.data = frame
        self.frame_mode_pub.publish(msg)

    # ── go to a stored point ─────────────────────────────────────────────
    def go_to_point(self, name, use_joint=True):
        """Ask moveit_go_to_pose to move to a point from points.yaml.

        The motion-type flag must go out first: the node refuses the command
        with "Select Joint or Cartesian mode" if neither flag is set, and it
        clears both flags after each move, so this is required every time.
        """
        flag = Bool()
        flag.data = True
        if use_joint:
            self.joint_motion_pub.publish(flag)
        else:
            self.cartesian_motion_pub.publish(flag)

        msg = String()
        msg.data = f"get_last_pose@{name}"
        self.ui_cmd_pub.publish(msg)

    # ── workspace calibration ────────────────────────────────────────────
    def publish_start_calibration(self, frame):
        msg = String()
        msg.data = frame
        self.start_calib_pub.publish(msg)

    def publish_go_to_frame_origin(self, frame):
        msg = String()
        msg.data = frame
        self.frame_origin_pub.publish(msg)

    def publish_finish_calibration(self):
        msg = Bool()
        msg.data = True
        self.finish_calib_pub.publish(msg)

    def publish_go_to_tf_point(self, frame, slot):
        """Publishes "{frame}_{slot}", e.g. "tf1_point1"."""
        msg = String()
        msg.data = f"{frame}_{slot}"
        self.go_to_tf_point_pub.publish(msg)


def load_overspeed_config():
    """Read the overspeed YAML, falling back to the built-in table."""
    try:
        with open(OVERSPEED_YAML, "r", encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)["imotor_overspeed_threshold_limits"]
        return cfg
    except Exception:
        return OVERSPEED_FALLBACK


class ModeSwitchWorker(QtCore.QObject):
    """Runs the Auto / SafeAuto / Jog sequence off the GUI thread.

    The sequence mirrors nextupWeb/ros/modeOverspeed.js:

      to Jog (drive 9)         : write SDOs, THEN publish /change_mode
      Jog to Auto/SafeAuto (8) : publish /change_mode, THEN write SDOs
      Auto <-> SafeAuto        : SDOs only, no /change_mode at all

    The order matters for safety. Going into Jog the speed limit must already
    be lowered before the drive is allowed to move, and coming out of Jog the
    drive mode must change before the limit is raised.
    """

    progress = QtCore.pyqtSignal(str, int)     # message, percent
    finished = QtCore.pyqtSignal(bool, str)    # ok, message

    def __init__(self, ros_node, mode, from_mode):
        super().__init__()
        self.ros = ros_node
        self.mode = mode
        self.from_mode = from_mode

    @QtCore.pyqtSlot()
    def run(self):
        cfg = load_overspeed_config()
        profile = cfg.get(self.mode)
        if profile is None:
            self.finished.emit(False, f"No overspeed profile for '{self.mode}'")
            return

        target_drive = MODE_DRIVE[self.mode]
        source_drive = MODE_DRIVE.get(self.from_mode)
        with_mode_change = source_drive != target_drive

        index = cfg.get("index", 0x200A)
        subindex = cfg.get("subindex", 0x09)
        dtype = cfg.get("type", "uint16")

        # Weighting matches the web UI so the progress bar behaves the same.
        service_weight = 10 if with_mode_change else (100 / 6)
        mode_weight = 40 if with_mode_change else 0
        pct = 0.0

        def do_sdos():
            nonlocal pct
            for i, joint in enumerate(JOINTS):
                value = profile[joint]
                self.progress.emit(f"{joint} -> {value}", int(pct))
                ok, msg = self.ros.write_overspeed(
                    i, index, subindex, dtype, value)
                if not ok:
                    self.finished.emit(
                        False, f"SDO write failed on {joint} (slave {i}): {msg}")
                    return False
                pct += service_weight
            return True

        def do_mode():
            nonlocal pct
            self.progress.emit(f"/change_mode {target_drive}", int(pct))
            self.ros.publish_change_mode(target_drive)
            pct += mode_weight
            return True

        if with_mode_change and target_drive == "9":
            ok = do_sdos() and do_mode()
        elif with_mode_change:
            ok = do_mode() and do_sdos()
        else:
            ok = do_sdos()

        if ok:
            self.progress.emit("complete", 100)
            self.finished.emit(True, f"Mode set to {self.mode}")


class PlannerWorker(QtCore.QObject):
    """Calls /change_planning_pipeline off the GUI thread."""

    finished = QtCore.pyqtSignal(bool, str, bool)   # ok, message, use_ompl

    def __init__(self, ros_node, use_ompl):
        super().__init__()
        self.ros = ros_node
        self.use_ompl = use_ompl

    @QtCore.pyqtSlot()
    def run(self):
        ok, msg = self.ros.set_planner(self.use_ompl)
        self.finished.emit(ok, msg, self.use_ompl)


def load_calibration():
    """Read calibration.yaml. A missing or empty file means 'no frames yet'."""
    try:
        with open(CALIBRATION_FILE, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        return data if isinstance(data, dict) else {}
    except FileNotFoundError:
        return {}
    except Exception:
        return {}


def save_calibration(data):
    """Atomic write, so a crash mid-save cannot truncate the file."""
    directory = os.path.dirname(CALIBRATION_FILE)
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp = CALIBRATION_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, sort_keys=False, default_flow_style=False)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, CALIBRATION_FILE)


def calib_timestamp():
    """DD-MM-YYYY HH:MM:SS, matching what the calibration node writes."""
    return datetime.now().strftime("%d-%m-%Y %H:%M:%S")


def load_saved_frame():
    try:
        with open(SERVO_FRAME_FILE, "r", encoding="utf-8") as fh:
            return json.load(fh).get("frame") or DEFAULT_FRAME
    except Exception:
        return DEFAULT_FRAME


def save_servo_frame(frame):
    try:
        directory = os.path.dirname(SERVO_FRAME_FILE)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(SERVO_FRAME_FILE, "w", encoding="utf-8") as fh:
            json.dump({"frame": frame}, fh, indent=2)
    except OSError:
        pass    # persistence is a convenience; never block the frame change


class ServoWorker(QtCore.QObject):
    """Calls /servo_node/start_servo off the GUI thread."""

    finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, ros_node):
        super().__init__()
        self.ros = ros_node

    @QtCore.pyqtSlot()
    def run(self):
        ok, msg = self.ros.start_servo()
        self.finished.emit(ok, msg)


class WsCalibrationDialog(QtWidgets.QDialog):
    """Workspace calibration.

    A frame is calibrated by teaching it four points. Each slot stores the
    pose the robot is at when you press Set, so the flow is: jog the arm to
    the corner, press Set, repeat for all four, then Start.

    Start is deliberately gated on all four slots being filled — that is the
    same guard the web backend applies, and calibrating from a partial set
    would produce a silently wrong frame.
    """

    def __init__(self, ros_node, get_pose, parent=None):
        super().__init__(parent)
        self.ros = ros_node
        self.get_pose = get_pose          # callable -> (joints, coordinate)
        self.data = load_calibration()
        self.current_frame = None

        self.setWindowTitle("Workspace Calibration")
        self.resize(880, 640)

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(16)

        # ── left: frames ────────────────────────────────────────────────
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(10)
        left.addWidget(self._title("Frames"))

        self.frameList = QtWidgets.QListWidget()
        self.frameList.setMinimumWidth(220)
        self.frameList.currentTextChanged.connect(self._on_frame_selected)
        left.addWidget(self.frameList, 1)

        self.newFrameInput = QtWidgets.QLineEdit()
        self.newFrameInput.setPlaceholderText("new frame name")
        self.newFrameInput.setMinimumHeight(46)
        left.addWidget(self.newFrameInput)

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        self.addFrameButton = QtWidgets.QPushButton("Add")
        self.addFrameButton.setMinimumHeight(48)
        self.addFrameButton.clicked.connect(self.add_frame)
        self.deleteFrameButton = QtWidgets.QPushButton("Delete")
        self.deleteFrameButton.setMinimumHeight(48)
        self.deleteFrameButton.setStyleSheet("background:#ef4444; color:#fff;")
        self.deleteFrameButton.clicked.connect(self.delete_frame)
        row.addWidget(self.addFrameButton)
        row.addWidget(self.deleteFrameButton)
        left.addLayout(row)
        root.addLayout(left)

        # ── right: slots + actions ──────────────────────────────────────
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(10)
        self.frameTitle = self._title("No frame selected")
        right.addWidget(self.frameTitle)

        self.slotWidgets = {}
        for slot in POINT_SLOTS:
            right.addWidget(self._slot_row(slot))

        right.addStretch(1)

        actions = QtWidgets.QHBoxLayout()
        actions.setSpacing(10)

        self.originButton = QtWidgets.QPushButton("Go to Origin")
        self.originButton.setMinimumHeight(52)
        self.originButton.clicked.connect(self.go_to_origin)

        self.startButton = QtWidgets.QPushButton("Start Calibration")
        self.startButton.setMinimumHeight(52)
        self.startButton.setStyleSheet("background:#3b82f6; color:#fff;")
        self.startButton.clicked.connect(self.start_calibration)

        self.finishButton = QtWidgets.QPushButton("Finish")
        self.finishButton.setMinimumHeight(52)
        self.finishButton.setStyleSheet("background:#22c55e; color:#fff;")
        self.finishButton.clicked.connect(self.finish_calibration)

        closeButton = QtWidgets.QPushButton("Close")
        closeButton.setMinimumHeight(52)
        closeButton.clicked.connect(self.accept)

        actions.addWidget(self.originButton)
        actions.addWidget(self.startButton)
        actions.addWidget(self.finishButton)
        actions.addStretch(1)
        actions.addWidget(closeButton)
        right.addLayout(actions)
        root.addLayout(right, 1)

        self._refresh_frames()

    # ── construction helpers ────────────────────────────────────────────
    @staticmethod
    def _title(text):
        label = QtWidgets.QLabel(text)
        label.setStyleSheet("font-size:18px; font-weight:700;")
        return label

    def _slot_row(self, slot):
        frame = QtWidgets.QFrame()
        frame.setStyleSheet(
            "background:#f8fafc; border:1px solid #e2e8f0; border-radius:12px;")
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(10)

        name = QtWidgets.QLabel(slot)
        name.setMinimumWidth(70)
        name.setStyleSheet("font-weight:700; border:none; background:transparent;")

        status = QtWidgets.QLabel("not set")
        status.setStyleSheet("color:#94a3b8; border:none; background:transparent;")

        setButton = QtWidgets.QPushButton("Set")
        setButton.setMinimumSize(84, 44)
        setButton.clicked.connect(lambda _, s=slot: self.set_slot(s))

        goButton = QtWidgets.QPushButton("Go")
        goButton.setMinimumSize(84, 44)
        goButton.clicked.connect(lambda _, s=slot: self.go_to_slot(s))

        layout.addWidget(name)
        layout.addWidget(status, 1)
        layout.addWidget(setButton)
        layout.addWidget(goButton)

        self.slotWidgets[slot] = (status, setButton, goButton)
        return frame

    # ── frame management ────────────────────────────────────────────────
    def _refresh_frames(self):
        self.frameList.clear()
        for name in sorted(self.data.keys()):
            self.frameList.addItem(name)
        if self.frameList.count():
            self.frameList.setCurrentRow(0)
        else:
            self.current_frame = None
            self._refresh_slots()

    def _on_frame_selected(self, name):
        self.current_frame = name or None
        self.frameTitle.setText(name or "No frame selected")
        self._refresh_slots()

    def _refresh_slots(self):
        frame_data = self.data.get(self.current_frame or "", {}) or {}
        for slot in POINT_SLOTS:
            status, setButton, goButton = self.slotWidgets[slot]
            point = frame_data.get(slot)
            enabled = self.current_frame is not None
            setButton.setEnabled(enabled)
            goButton.setEnabled(enabled and point is not None)
            if point:
                status.setText(
                    f"x {point.get('x', 0):.1f}  y {point.get('y', 0):.1f}  "
                    f"z {point.get('z', 0):.1f}   ({point.get('updated', '')})")
                status.setStyleSheet(
                    "color:#16a34a; border:none; background:transparent;")
            else:
                status.setText("not set")
                status.setStyleSheet(
                    "color:#94a3b8; border:none; background:transparent;")

        complete = self._is_complete()
        self.startButton.setEnabled(complete)
        self.originButton.setEnabled(self.current_frame is not None)
        self.startButton.setToolTip(
            "" if complete else "All four points must be set first")

    def _is_complete(self):
        if not self.current_frame:
            return False
        frame_data = self.data.get(self.current_frame, {}) or {}
        return all(isinstance(frame_data.get(s), dict) for s in POINT_SLOTS)

    def add_frame(self):
        name = self.newFrameInput.text().strip()
        if not name:
            return
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
            QtWidgets.QMessageBox.warning(
                self, "Invalid name",
                "Frame names may contain letters, numbers, dash and underscore only.")
            return
        if name in self.data:
            QtWidgets.QMessageBox.warning(
                self, "Duplicate frame", f"'{name}' already exists.")
            return

        self.data[name] = {}
        if not self._save():
            return
        self.newFrameInput.clear()
        self._refresh_frames()
        self.frameList.setCurrentItem(
            self.frameList.findItems(name, QtCore.Qt.MatchExactly)[0])

    def delete_frame(self):
        if not self.current_frame:
            return
        answer = QtWidgets.QMessageBox.question(
            self, "Delete frame",
            f"Delete frame '{self.current_frame}' and all four of its points?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self.data.pop(self.current_frame, None)
        if self._save():
            self._refresh_frames()

    # ── slot management ─────────────────────────────────────────────────
    def set_slot(self, slot):
        if not self.current_frame:
            return
        joints, coordinate = self.get_pose()
        frame_data = self.data.setdefault(self.current_frame, {})
        existing = frame_data.get(slot) or {}

        # calibration.yaml spells the angles out; points.yaml uses r/p/w.
        point = {
            "x": coordinate.get("x", 0.0),
            "y": coordinate.get("y", 0.0),
            "z": coordinate.get("z", 0.0),
            "roll": coordinate.get("r", 0.0),
            "pitch": coordinate.get("p", 0.0),
            "yaw": coordinate.get("w", 0.0),
            "joints": joints,
        }
        # created is preserved across re-sets; updated always bumps.
        point["created"] = existing.get("created", calib_timestamp())
        point["updated"] = calib_timestamp()

        frame_data[slot] = point
        if self._save():
            self._refresh_slots()

    def go_to_slot(self, slot):
        if not self.current_frame:
            return
        self.ros.publish_go_to_tf_point(self.current_frame, slot)

    # ── publishes ───────────────────────────────────────────────────────
    def go_to_origin(self):
        if self.current_frame:
            self.ros.publish_go_to_frame_origin(self.current_frame)

    def start_calibration(self):
        if not self._is_complete():
            QtWidgets.QMessageBox.warning(
                self, "Incomplete frame",
                "All four points must be set before starting calibration.")
            return
        self.ros.publish_start_calibration(self.current_frame)

    def finish_calibration(self):
        self.ros.publish_finish_calibration()

    # ── persistence ─────────────────────────────────────────────────────
    def _save(self):
        try:
            save_calibration(self.data)
            return True
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, "Save failed",
                f"Could not write calibration.yaml.\n\n{exc}")
            self.data = load_calibration()
            self._refresh_slots()
            return False


class HomeWorker(QtCore.QObject):
    """The HOME sequence, mirroring wsHandler.js.

      1. write the SafeAuto overspeed profile to all 6 drives (continue even
         if some fail — homing proceeds regardless)
      2. publish /change_mode 8
      3. wait up to 3s for /nextup_joint_states to confirm mode 8
      4. settle, then publish "home" on /ui_commands

    The settle in step 4 is the subtle part. A confirmed 9->8 transition proves
    the drive arrived, so it needs only a short guard. A drive ALREADY at 8
    gives no confirmation edge — wait_for_drive_mode returns instantly — so it
    needs the full blind wait. If the 3s wait timed out, that time has already
    elapsed and no further settle is needed.
    """

    progress = QtCore.pyqtSignal(str, int)
    finished = QtCore.pyqtSignal(bool, str)

    BLIND_SETTLE_S = 2.0        # already at mode 8, no feedback to rely on
    CONFIRMED_GUARD_S = 0.5     # feedback confirmed a real 9 -> 8 transition
    MODE_TIMEOUT_S = 3.0

    def __init__(self, ros_node):
        super().__init__()
        self.ros = ros_node

    @QtCore.pyqtSlot()
    def run(self):
        cfg = load_overspeed_config()
        profile = cfg.get("safeauto")
        if profile is None:
            self.finished.emit(False, "No safeauto overspeed profile")
            return

        index = cfg.get("index", 0x200A)
        subindex = cfg.get("subindex", 0x09)
        dtype = cfg.get("type", "uint16")

        # 1) Overspeed first, best-effort.
        overspeed_ok = True
        failures = []
        for i, joint in enumerate(JOINTS):
            self.progress.emit(f"overspeed {joint}", int(i / 6 * 40))
            ok, msg = self.ros.write_overspeed(
                i, index, subindex, dtype, profile[joint])
            if not ok:
                overspeed_ok = False
                failures.append(f"{joint}: {msg}")

        # 2) Change to drive mode 8. Capture where we are coming FROM, because
        #    the settle below depends on whether this is a real transition.
        with self.ros._status_lock:
            prior_mode = self.ros.drive_mode
        self.progress.emit("change_mode 8", 45)
        self.ros.publish_change_mode("8")

        # 3) Confirm via feedback.
        confirmed, waited = self.ros.wait_for_drive_mode(8, self.MODE_TIMEOUT_S)
        self.progress.emit(
            "mode 8 confirmed" if confirmed else "mode 8 unconfirmed", 70)

        # 4) Settle.
        if not confirmed:
            settle = 0.0
        elif prior_mode == 9:
            settle = self.CONFIRMED_GUARD_S
        else:
            settle = max(0.0, self.BLIND_SETTLE_S - waited)

        if settle > 0:
            self.progress.emit(f"settling {settle:.1f}s", 85)
            time.sleep(settle)

        self.ros.publish_ui_command("home")
        self.progress.emit("home published", 100)

        if overspeed_ok:
            message = ("Homing started (SafeAuto)" if confirmed
                       else "Homing started (SafeAuto, mode unconfirmed)")
        else:
            message = ("Homing started; overspeed incomplete: "
                       + "; ".join(failures))
        self.finished.emit(True, message)


class ResetWorker(QtCore.QObject):
    """The RESET sequence, mirroring wsHandler.js.

      1. abort any running trajectory
      2. trigger emergency (this is what actually clears the fault latch)
      3. wait 1.5s, publish /reset_fault
      4. wait 2.5s for the reset state machine to settle
      5. reset lands the drive in mode 9, so write the Jog overspeed profile

    The 1.5s and 2.5s gaps are the drive's, not ours — shortening them means
    writing thresholds into a drive that is still resetting.
    """

    progress = QtCore.pyqtSignal(str, int)
    finished = QtCore.pyqtSignal(bool, str)

    RESET_FAULT_DELAY_S = 1.5
    RESET_SETTLE_S = 2.5

    def __init__(self, ros_node):
        super().__init__()
        self.ros = ros_node

    @QtCore.pyqtSlot()
    def run(self):
        self.progress.emit("stopping trajectory", 5)
        self.ros.stop_moveit()

        self.progress.emit("triggering emergency", 10)
        if not self.ros.trigger_emergency():
            self.finished.emit(
                False, "nextup_joint_interfaces not installed — "
                       "cannot publish the emergency trigger")
            return

        self.progress.emit("waiting before reset_fault", 20)
        time.sleep(self.RESET_FAULT_DELAY_S)

        self.progress.emit("/reset_fault", 35)
        self.ros.publish_reset_fault()

        self.progress.emit("waiting for reset to settle", 45)
        time.sleep(self.RESET_SETTLE_S)

        cfg = load_overspeed_config()
        profile = cfg.get("jog")
        if profile is None:
            self.finished.emit(True, "Reset complete (no jog overspeed profile)")
            return

        index = cfg.get("index", 0x200A)
        subindex = cfg.get("subindex", 0x09)
        dtype = cfg.get("type", "uint16")

        failures = []
        for i, joint in enumerate(JOINTS):
            self.progress.emit(f"overspeed {joint}", 55 + int(i / 6 * 45))
            ok, msg = self.ros.write_overspeed(
                i, index, subindex, dtype, profile[joint])
            if not ok:
                failures.append(f"{joint}: {msg}")

        self.progress.emit("complete", 100)
        if failures:
            self.finished.emit(
                True, "Reset complete; overspeed incomplete: " + "; ".join(failures))
        else:
            self.finished.emit(True, "Reset complete (Jog)")


class YamlViewerDialog(QtWidgets.QDialog):
    """Small read-only window showing the points YAML."""

    def __init__(self, path, parent=None):
        super().__init__(parent)
        self.path = path
        self.setWindowTitle(f"Points YAML — {os.path.basename(path)}")
        self.resize(760, 620)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        self.pathLabel = QtWidgets.QLabel(path)
        self.pathLabel.setStyleSheet("color:#64748b; font-size:12px;")
        self.pathLabel.setWordWrap(True)
        layout.addWidget(self.pathLabel)

        self.editor = QtWidgets.QPlainTextEdit()
        self.editor.setReadOnly(True)
        self.editor.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.editor.setStyleSheet(
            "background:#0f172a; color:#e2e8f0; border-radius:10px;"
            "font-family:'DejaVu Sans Mono',monospace; font-size:13px; padding:12px;"
        )
        layout.addWidget(self.editor, 1)

        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(10)

        self.reloadButton = QtWidgets.QPushButton("Reload")
        self.reloadButton.setMinimumHeight(46)
        self.reloadButton.clicked.connect(self.load)

        self.copyButton = QtWidgets.QPushButton("Copy")
        self.copyButton.setMinimumHeight(46)
        self.copyButton.clicked.connect(self.copy_to_clipboard)

        self.closeButton = QtWidgets.QPushButton("Close")
        self.closeButton.setMinimumHeight(46)
        self.closeButton.clicked.connect(self.accept)

        buttons.addWidget(self.reloadButton)
        buttons.addWidget(self.copyButton)
        buttons.addStretch(1)
        buttons.addWidget(self.closeButton)
        layout.addLayout(buttons)

        self.load()

    def load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                self.editor.setPlainText(fh.read())
        except OSError as exc:
            self.editor.setPlainText(f"Could not read file:\n{exc}")

    def copy_to_clipboard(self):
        QtWidgets.QApplication.clipboard().setText(self.editor.toPlainText())


class PointRow(QtWidgets.QWidget):
    """One row of the point list."""

    executeRequested = QtCore.pyqtSignal(int)
    editRequested = QtCore.pyqtSignal(int)
    deleteRequested = QtCore.pyqtSignal(int)

    def __init__(self, index, name, parent=None):
        super().__init__(parent)
        self.point_index = index

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(8)

        badge = QtWidgets.QLabel(str(index + 1))
        badge.setAlignment(QtCore.Qt.AlignCenter)
        badge.setFixedSize(34, 34)
        badge.setStyleSheet(
            "background:#eff6ff; color:#2563eb; border-radius:17px;"
            "font-weight:700; font-size:13px;"
        )
        layout.addWidget(badge)

        label = QtWidgets.QLabel(name)
        label.setStyleSheet(
            "background:#eff6ff; color:#2563eb; border-radius:8px;"
            "padding:7px 14px; font-weight:600;"
        )
        layout.addWidget(label)
        layout.addStretch(1)

        for text, colour, signal in (
            ("\u27a4", "#22c55e", self.executeRequested),
            ("\u270e", "#f59e0b", self.editRequested),
            ("\u2716", "#ef4444", self.deleteRequested),
        ):
            btn = QtWidgets.QPushButton(text)
            btn.setFixedSize(44, 44)
            btn.setCursor(QtCore.Qt.PointingHandCursor)
            btn.setStyleSheet(
                f"background:{colour}; color:#ffffff; border:none;"
                "border-radius:10px; font-size:16px;"
            )
            btn.clicked.connect(lambda checked=False, s=signal:
                                s.emit(self.point_index))
            layout.addWidget(btn)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        uic.loadUi(UI_FILE, self)

        self.points = []
        self.points_file_name = ""
        self.selected_point_index = None
        self.is_loading_point = False

        self._setup_nav()
        self._setup_leds()
        self._setup_jog()
        self._setup_clock()
        # Points must load before the frame selector, which is built from the
        # is_tf points.
        self._setup_point_functionality()
        self._setup_frames()
        self._setup_planner_and_mode()
        self._setup_actions()
        self._setup_ros_timer()

        self.bodyLayout.setStretch(0, 3)
        self.bodyLayout.setStretch(1, 3)
        self.bodyLayout.setStretch(2, 5)

    def _setup_ros_timer(self):
        """Set up a timer to update UI from ROS data."""
        self._ros_timer = QtCore.QTimer(self)
        self._ros_timer.timeout.connect(self._update_from_ros)
        self._ros_timer.start(50)

    def _update_from_ros(self):
        """Update UI displays from ROS data. STRICT RAW DISPLAY."""
        # Status LEDs update regardless of whether a point is being loaded —
        # they are safety indicators and must never be stale.
        self._refresh_status()

        if self.is_loading_point:
            return

        if self.ros_node.joint_data is not None:
            data = self.ros_node.joint_data
            for i in range(min(6, len(data))):
                # NO MATH. Just display the raw data.
                self._write_display_value("joint", i + 1, data[i])

        if self.ros_node.cart_data is not None:
            data = self.ros_node.cart_data
            for i in range(min(6, len(data))):
                # NO MATH. Just display the raw data.
                self._write_display_value("cart", i + 1, data[i])

    # ── point YAML storage ────────────────────────────────────────────────

    def _setup_point_functionality(self):
        self.pointList.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection
        )

        self.pointList.itemSelectionChanged.connect(self._on_list_selection)
        self.addPointButton.clicked.connect(self.add_point)
        self.updatePointButton.clicked.connect(self.update_point)
        self.reloadPointsButton.clicked.connect(self.load_points)
        self.viewYamlButton.clicked.connect(self.view_yaml)
        self.deleteAllButton.clicked.connect(self.delete_all_points)
        self.undoButton.clicked.connect(self.clear_editor)
        self.pointList.itemDoubleClicked.connect(lambda item: self._edit_list_item(item))

        self.load_points()

    def _read_points_file(self):
        if not os.path.isfile(POINTS_FILE):
            raise FileNotFoundError(f"Points file does not exist:\n{POINTS_FILE}")

        with open(POINTS_FILE, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}

        if not isinstance(data, dict):
            raise ValueError("points.yaml root must be a YAML mapping.")

        points = data.get("points", [])
        if points is None:
            points = []

        if not isinstance(points, list):
            raise ValueError("'points' in points.yaml must be a list.")

        self.points_file_name = data.get("points_file_name", "")
        return data, points

    def _write_points_file(self, points):
        data, _ = self._read_points_file()
        data["points"] = points

        directory = os.path.dirname(POINTS_FILE)
        os.makedirs(directory, exist_ok=True)

        temp_file = POINTS_FILE + ".tmp"

        with open(temp_file, "w", encoding="utf-8") as fh:
            yaml.safe_dump(data, fh, sort_keys=False, allow_unicode=True, default_flow_style=False)

        os.replace(temp_file, POINTS_FILE)

    def load_points(self):
        try:
            data, points = self._read_points_file()
            self.points = deepcopy(points)
            self.points_file_name = data.get("points_file_name", "")
            self._refresh_point_list()

            self.statusbar.showMessage(f"Loaded {len(self.points)} points from {POINTS_FILE}", 3000)
        except Exception as exc:
            self.points = []
            self._refresh_point_list()
            QtWidgets.QMessageBox.critical(self, "Unable to load points", f"Could not read points.yaml.\n\n{exc}")

    def _refresh_point_list(self):
        self.pointList.blockSignals(True)
        self.pointList.clear()

        for index, point in enumerate(self.points):
            name = str(point.get("name", f"point-{index + 1}"))
            item = QtWidgets.QListWidgetItem()
            row = PointRow(index, name)

            item.setSizeHint(QtCore.QSize(0, 62))
            row.executeRequested.connect(self.execute_point)
            row.editRequested.connect(self.edit_point)
            row.deleteRequested.connect(self.delete_point)

            self.pointList.addItem(item)
            self.pointList.setItemWidget(item, row)

        self.pointList.blockSignals(False)
        self.pointCountLabel.setText(f"{len(self.points)} points")

        if self.selected_point_index is not None:
            if 0 <= self.selected_point_index < len(self.points):
                self.pointList.setCurrentRow(self.selected_point_index)
            else:
                self.selected_point_index = None

        # The is_tf set may have changed, so the frame selectors follow.
        # Guarded because this runs once before _setup_frames has built them.
        if hasattr(self, "current_frame"):
            self._refresh_servo_frames()

    def _on_list_selection(self):
        row = self.pointList.currentRow()
        if 0 <= row < len(self.points):
            self.selected_point_index = row
            self._populate_editor(self.points[row])

    def _edit_list_item(self, item):
        row = self.pointList.row(item)
        if 0 <= row < len(self.points):
            self.edit_point(row)

    # ── editor ─────────────────────────────────────────────────────────────

    def _populate_editor(self, point):
        self.pointNameInput.setText(str(point.get("name", "")))
        self.sequenceInput.setValue(int(point.get("sequence", 1) or 1))
        self.enableTfCheck.setChecked(bool(point.get("is_tf", False)))
        self.editableCheck.setChecked(bool(point.get("is_editable", True)))

        nature = point.get("nature", "")
        self.natureInput.setPlainText("" if nature is None else str(nature))

        date_time = point.get("date_time", "")
        self.dateTimeInput.setText("" if date_time is None else str(date_time))

        self._set_position_values_from_point(point)

    # ── position values shown beside +/- buttons ──────────────────────────

    def _read_display_value(self, prefix, index):
        label = getattr(self, f"{prefix}Value{index}")
        text = label.text().strip().replace("°", "").replace("cm", "")
        try:
            return float(text)
        except ValueError:
            return 0.0

    def _write_display_value(self, prefix, index, value):
        label = getattr(self, f"{prefix}Value{index}")
        # JUST SHOW THE RAW FLOAT NO MATTER THE UNIT
        label.setText(f"{value:.2f}")

    def _set_position_values_from_point(self, point):
        self.is_loading_point = True
        
        joints = point.get("joints_values") or {}
        for i in range(1, 7):
            # NO MATH. Read raw YAML value.
            self._write_display_value("joint", i, float(joints.get(f"joint{i}", 0) or 0))

        coord = point.get("coordinate") or {}
        for i, axis in enumerate(("x", "y", "z", "r", "p", "w"), start=1):
            # NO MATH. Read raw YAML value.
            self._write_display_value("cart", i, float(coord.get(axis, 0) or 0))
            
        self.is_loading_point = False

    def _current_position_values(self):
        joints = {}
        for i in range(1, 7):
            # NO MATH. Save the exact UI float value.
            joints[f"joint{i}"] = self._read_display_value("joint", i)

        axes = ("x", "y", "z", "r", "p", "w")
        coordinate = {
            axis: self._read_display_value("cart", i)
            for i, axis in enumerate(axes, start=1)
        }
        return joints, coordinate

    def clear_editor(self):
        self.selected_point_index = None
        self.pointList.clearSelection()
        self.pointNameInput.clear()
        self.sequenceInput.setValue(1)
        self.enableTfCheck.setChecked(False)
        self.editableCheck.setChecked(True)
        self.natureInput.clear()
        self.dateTimeInput.clear()
        self.statusbar.showMessage("Point editor cleared", 1500)

    def _editor_values(self):
        name = self.pointNameInput.text().strip()
        nature = self.natureInput.toPlainText().strip()

        if not name:
            raise ValueError("Point name cannot be empty.")

        if any(
            str(point.get("name", "")).strip() == name
            and i != self.selected_point_index
            for i, point in enumerate(self.points)
        ):
            raise ValueError(f"A point named '{name}' already exists.")

        return {
            "name": name,
            "sequence": self.sequenceInput.value(),
            "is_tf": self.enableTfCheck.isChecked(),
            "is_editable": self.editableCheck.isChecked(),
            "nature": nature,
        }

    def add_point(self):
        try:
            values = self._editor_values()
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid point", str(exc))
            return

        point = {
            "name": values["name"],
            "date_time": datetime.now().strftime("%d%b_%H%M").lower(),
            "sequence": values["sequence"],
            "nature": values["nature"],
            "is_tf": values["is_tf"],
            "is_calibrated": False,
            "is_editable": values["is_editable"],
        }

        point["joints_values"], point["coordinate"] = self._current_position_values()

        try:
            self.points.append(point)
            self._write_points_file(self.points)
            new_index = len(self.points) - 1
            self.selected_point_index = new_index
            self._refresh_point_list()
            self.pointList.setCurrentRow(new_index)
            self.statusbar.showMessage(f"Added point '{point['name']}'", 2500)
        except Exception as exc:
            self.load_points()
            QtWidgets.QMessageBox.critical(self, "Save failed", f"Could not save the new point.\n\n{exc}")

    def update_point(self):
        index = self.selected_point_index
        if index is None or not (0 <= index < len(self.points)):
            QtWidgets.QMessageBox.information(self, "No point selected", "Select a point to update.")
            return

        original = self.points[index]
        if original.get("is_editable", True) is False:
            QtWidgets.QMessageBox.warning(self, "Point locked", f"'{original.get('name', 'point')}' is marked as not editable.")
            return

        try:
            values = self._editor_values()
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid point", str(exc))
            return

        updated = deepcopy(original)
        updated["name"] = values["name"]
        updated["sequence"] = values["sequence"]
        updated["nature"] = values["nature"]
        updated["is_tf"] = values["is_tf"]
        updated["is_editable"] = values["is_editable"]
        updated["joints_values"], updated["coordinate"] = self._current_position_values()

        self.points[index] = updated

        try:
            self._write_points_file(self.points)
            self._refresh_point_list()
            self.pointList.setCurrentRow(index)
            self.statusbar.showMessage(f"Updated point '{updated['name']}'", 2500)
        except Exception as exc:
            self.load_points()
            QtWidgets.QMessageBox.critical(self, "Save failed", f"Could not update the point.\n\n{exc}")

    def edit_point(self, index):
        if not (0 <= index < len(self.points)):
            return
        self.selected_point_index = index
        self.pointList.setCurrentRow(index)
        self._populate_editor(self.points[index])
        self.statusbar.showMessage(f"Editing point '{self.points[index].get('name', '')}'", 2000)

    def execute_point(self, index):
        """Move the robot to a stored point.

        moveit_go_to_pose reads the joint values straight out of points.yaml,
        so only the name goes over the wire. It needs a motion type each time,
        which is what the prompt is for.
        """
        if not (0 <= index < len(self.points)):
            return
        point = self.points[index]
        name = str(point.get("name", ""))
        if not name:
            return

        self.selected_point_index = index
        self.pointList.setCurrentRow(index)
        self._populate_editor(point)

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("Move to point")
        box.setText(f"Move to '{name}'?")
        box.setInformativeText("Choose how the robot should plan the motion.")
        joint_btn = box.addButton("Joint", QtWidgets.QMessageBox.AcceptRole)
        cart_btn = box.addButton("Cartesian", QtWidgets.QMessageBox.AcceptRole)
        box.addButton("Cancel", QtWidgets.QMessageBox.RejectRole)
        box.exec_()

        clicked = box.clickedButton()
        if clicked not in (joint_btn, cart_btn):
            return

        use_joint = clicked is joint_btn
        self.ros_node.go_to_point(name, use_joint=use_joint)
        self.statusbar.showMessage(
            f"Moving to '{name}' ({'joint' if use_joint else 'cartesian'})", 4000)

    def delete_point(self, index):
        if not (0 <= index < len(self.points)):
            return
        point = self.points[index]
        name = str(point.get("name", f"point-{index + 1}"))

        if point.get("is_editable", True) is False:
            QtWidgets.QMessageBox.warning(self, "Point locked", f"'{name}' is marked as not editable.")
            return

        answer = QtWidgets.QMessageBox.question(
            self, "Delete point", f"Delete point '{name}'?\n\nThis will update points.yaml.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
        )

        if answer != QtWidgets.QMessageBox.Yes:
            return

        removed = self.points.pop(index)
        try:
            self._write_points_file(self.points)
            self.selected_point_index = None
            self._refresh_point_list()
            self.clear_editor()
            self.statusbar.showMessage(f"Deleted point '{removed.get('name', name)}'", 2500)
        except Exception as exc:
            self.load_points()
            QtWidgets.QMessageBox.critical(self, "Delete failed", f"Could not delete the point.\n\n{exc}")

    def delete_all_points(self):
        if not self.points:
            return

        answer = QtWidgets.QMessageBox.question(
            self, "Delete all points", "Delete all editable points from points.yaml?\n\nNon-editable/locked points will be kept.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
        )

        if answer != QtWidgets.QMessageBox.Yes:
            return

        kept = [point for point in self.points if point.get("is_editable", True) is False]
        deleted_count = len(self.points) - len(kept)
        self.points = kept

        try:
            self._write_points_file(self.points)
            self.selected_point_index = None
            self._refresh_point_list()
            self.clear_editor()
            self.statusbar.showMessage(f"Deleted {deleted_count} editable points", 2500)
        except Exception as exc:
            self.load_points()
            QtWidgets.QMessageBox.critical(self, "Delete failed", f"Could not delete points.\n\n{exc}")

    def view_yaml(self):
        if not os.path.isfile(POINTS_FILE):
            QtWidgets.QMessageBox.warning(self, "Points YAML not found", POINTS_FILE)
            return
        dialog = YamlViewerDialog(POINTS_FILE, self)
        dialog.exec_()

    # ── planner + mode ───────────────────────────────────────────────────
    def _setup_planner_and_mode(self):
        """Wire OMPL/PILZ and SafeAuto/Jog/Auto.

        Both run their ROS work on a worker thread. The buttons are disabled
        for the duration so a second switch cannot start while the first is
        still writing SDOs.
        """
        self.current_planner = "OMPL" if self.omplButton.isChecked() else "PILZ"
        self.current_mode = "jog"          # matches the .ui default
        self._thread = None
        self._worker = None

        self.omplButton.clicked.connect(lambda: self.request_planner(True))
        self.pilzButton.clicked.connect(lambda: self.request_planner(False))

        self.safeAutoButton.clicked.connect(lambda: self.request_mode("safeauto"))
        self.jogModeButton.clicked.connect(lambda: self.request_mode("jog"))
        self.autoModeButton.clicked.connect(lambda: self.request_mode("auto"))

    def _busy(self, busy):
        for btn in (self.omplButton, self.pilzButton, self.safeAutoButton,
                    self.jogModeButton, self.autoModeButton):
            btn.setEnabled(not busy)

    def _set_planner_buttons(self, planner):
        self.omplButton.setChecked(planner == "OMPL")
        self.pilzButton.setChecked(planner == "PILZ")

    def _set_mode_buttons(self, mode):
        self.safeAutoButton.setChecked(mode == "safeauto")
        self.jogModeButton.setChecked(mode == "jog")
        self.autoModeButton.setChecked(mode == "auto")

    def _run_worker(self, worker, on_finished):
        thread = QtCore.QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(on_finished)
        worker.finished.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        # Keep references, otherwise Python garbage-collects the thread
        # mid-run and the app dies with no traceback.
        self._thread = thread
        self._worker = worker
        thread.start()

    # planner ---------------------------------------------------------------
    def request_planner(self, use_ompl):
        wanted = "OMPL" if use_ompl else "PILZ"
        if wanted == self.current_planner:
            self._set_planner_buttons(self.current_planner)
            return

        self._busy(True)
        self.statusbar.showMessage(f"Switching planner to {wanted}...")
        self._run_worker(PlannerWorker(self.ros_node, use_ompl),
                         self._planner_finished)

    def _planner_finished(self, ok, message, use_ompl):
        self._busy(False)
        if ok:
            # Only adopt the new planner once the node confirms the switch,
            # so the buttons never show a pipeline that isn't actually active.
            self.current_planner = "OMPL" if use_ompl else "PILZ"
            self.statusbar.showMessage(f"Planner: {self.current_planner}", 3000)
        else:
            self.statusbar.showMessage(f"Planner switch failed: {message}", 6000)
            QtWidgets.QMessageBox.warning(self, "Planner switch failed", message)
        self._set_planner_buttons(self.current_planner)

    # mode ------------------------------------------------------------------
    def request_mode(self, mode):
        if mode == self.current_mode:
            self._set_mode_buttons(self.current_mode)
            return

        self._busy(True)
        self.statusbar.showMessage(f"Switching mode to {mode}...")

        worker = ModeSwitchWorker(self.ros_node, mode, self.current_mode)
        worker.progress.connect(self._mode_progress)
        self._pending_mode = mode
        self._run_worker(worker, self._mode_finished)

    def _mode_progress(self, message, percent):
        self.statusbar.showMessage(f"{percent}% — {message}")

    def _mode_finished(self, ok, message):
        self._busy(False)
        if ok:
            self.current_mode = self._pending_mode
            self.statusbar.showMessage(message, 3000)
        else:
            # The sequence aborted partway. The drive may be in a mixed state,
            # so say so rather than quietly reverting the buttons.
            self.statusbar.showMessage(f"Mode switch failed: {message}", 8000)
            QtWidgets.QMessageBox.warning(
                self, "Mode switch failed",
                f"{message}\n\nThe drives may be partially configured. "
                "Check the robot state before moving.")
        self._set_mode_buttons(self.current_mode)

    # ── servo + calibration ──────────────────────────────────────────────
    def _setup_actions(self):
        self.jogServoButton.clicked.connect(self.start_servo)
        self.wsCalibrationButton.clicked.connect(self.open_ws_calibration)

        # ws_sim runs as a child process, not a widget. See toggle_ws_sim.
        self.ws_sim_process = None
        self.nexSimButton.setCheckable(True)
        self.nexSimButton.clicked.connect(self.toggle_ws_sim)

        self.keyboardPendantButton.clicked.connect(self.toggle_keyboard_pendant)
        self._pendant_active = False

    # ── ws_sim (NexSim) ──────────────────────────────────────────────────
    def _ws_sim_preflight(self):
        """Return a human-readable reason ws_sim cannot run, or None.

        Checked before launching, because a child that dies during import
        looks exactly like "nothing happened" from the user's side.
        """
        if not os.path.isfile(WS_SIM_SCRIPT):
            return (f"ws_sim.py not found at:\n{WS_SIM_SCRIPT}\n\n"
                    "It should sit next to main.py, or change WS_SIM_SCRIPT.")

        if not os.path.isdir(NEXTUP_HMI_DIR):
            return (f"nextup_hmi checkout not found at:\n{NEXTUP_HMI_DIR}\n\n"
                    "Clone it:\n  git clone "
                    f"https://github.com/Slyder7634/nextup_hmi.git {NEXTUP_HMI_DIR}"
                    "\n\nOr change NEXTUP_HMI_DIR at the top of main.py.")

        if not os.path.isfile(os.path.join(NEXTUP_HMI_DIR, "viewer_widget.py")):
            return (f"{NEXTUP_HMI_DIR} exists but contains no viewer_widget.py.\n\n"
                    "Is NEXTUP_HMI_DIR pointing at the right checkout?")

        # The child runs under this same interpreter, so checking here names
        # the missing package instead of leaving a silent exit.
        missing = [m for m in ("PyQt6", "vtk")
                   if importlib.util.find_spec(m) is None]
        if missing:
            names = " ".join(missing)
            return (f"Missing Python package(s) for:\n  {sys.executable}\n\n"
                    f"  {names}\n\nInstall with:\n  pip install {names}")

        return None

    def toggle_ws_sim(self):
        """Start or stop the ws_sim viewer process.

        Separate process rather than an embedded widget: nextup_hmi is PyQt6
        and this app is PyQt5, so one process cannot host both Qt builds; and
        VTK plus a TF listener costs CPU that only dies with the process.
        """
        if self.ws_sim_process is not None:
            self.stop_ws_sim()
            return

        problem = self._ws_sim_preflight()
        if problem:
            self.nexSimButton.setChecked(False)
            QtWidgets.QMessageBox.warning(self, "Cannot start ws_sim", problem)
            return

        process = QtCore.QProcess(self)
        process.setProgram(sys.executable)
        # -u keeps the child unbuffered so a crash message reaches us before
        # the pipe is torn down.
        process.setArguments(["-u", WS_SIM_SCRIPT])
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._ws_sim_output)
        process.finished.connect(self._ws_sim_finished)
        process.errorOccurred.connect(self._ws_sim_error)

        env = QtCore.QProcessEnvironment.systemEnvironment()
        if NEXTUP_HMI_DIR:
            env.insert("NEXTUP_HMI_DIR", NEXTUP_HMI_DIR)
        process.setProcessEnvironment(env)

        self._ws_sim_log = []
        self._ws_sim_started_at = time.monotonic()
        process.start()
        if not process.waitForStarted(3000):
            self.nexSimButton.setChecked(False)
            QtWidgets.QMessageBox.warning(
                self, "ws_sim failed to start", process.errorString())
            return

        self.ws_sim_process = process
        self.nexSimButton.setChecked(True)
        self.statusbar.showMessage("ws_sim starting...", 3000)

    def _ws_sim_output(self):
        process = self.ws_sim_process
        if process is None:
            return
        text = bytes(process.readAllStandardOutput()).decode(
            "utf-8", errors="replace")
        if text.strip():
            self._ws_sim_log.append(text)
            print(f"[ws_sim] {text.rstrip()}", flush=True)

    def stop_ws_sim(self):
        process = self.ws_sim_process
        if process is None:
            return

        # Disconnect first: terminate() fires finished(), and that handler
        # must not re-enter while we are already tearing down.
        try:
            process.finished.disconnect(self._ws_sim_finished)
        except (TypeError, RuntimeError):
            pass

        process.terminate()                 # SIGTERM — ws_sim shuts ROS down
        if not process.waitForFinished(4000):
            process.kill()                  # ignored us; take it down hard
            process.waitForFinished(1000)

        self.ws_sim_process = None
        self.nexSimButton.setChecked(False)
        self.statusbar.showMessage("ws_sim stopped", 3000)

    def _ws_sim_finished(self, code, _status):
        """The viewer exited on its own — window closed, or it died."""
        self._ws_sim_output()               # drain anything still buffered
        self.ws_sim_process = None
        self.nexSimButton.setChecked(False)

        lifetime = time.monotonic() - getattr(self, "_ws_sim_started_at", 0.0)
        log = "".join(getattr(self, "_ws_sim_log", [])).strip()

        # Clean exit after a while means the user closed the window. A
        # non-zero code, or any exit within ~2s, means it never came up — and
        # that must not be a status-bar message nobody reads.
        if code == 0 and lifetime > 2.0:
            self.statusbar.showMessage("ws_sim closed", 3000)
            return

        self.statusbar.showMessage(f"ws_sim exited with code {code}", 8000)
        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Warning)
        box.setWindowTitle("ws_sim exited")
        box.setText(f"ws_sim stopped after {lifetime:.1f}s (exit code {code}).")
        box.setInformativeText("Output from the viewer process is below.")
        box.setDetailedText(log or "(no output)")
        box.exec_()

    def _ws_sim_error(self, _error):
        process = self.ws_sim_process
        if process is None:
            return
        self.statusbar.showMessage(f"ws_sim error: {process.errorString()}", 6000)
    # ── keyboard pendant ─────────────────────────────────────────────────
    def toggle_keyboard_pendant(self):
        """Jog from the keyboard: 1-6 pick a joint, arrows drive it.

        Installs an application-wide event filter so the keys work wherever
        focus happens to be. Off by default because a stray keypress moving
        the arm is exactly the kind of surprise a pendant should not spring.
        """
        self._pendant_active = not self._pendant_active
        self.keyboardPendantButton.setChecked(self._pendant_active)

        if self._pendant_active:
            QtWidgets.QApplication.instance().installEventFilter(self)
            self.statusbar.showMessage(
                "Keyboard pendant ON — 1-6 select joint, Up/Down to jog", 5000)
        else:
            QtWidgets.QApplication.instance().removeEventFilter(self)
            self.jog_release()
            self.statusbar.showMessage("Keyboard pendant OFF", 3000)

    def eventFilter(self, obj, event):
        if not self._pendant_active:
            return super().eventFilter(obj, event)

        # Ignore auto-repeat: the jog command is already repeating on its own
        # timer, and a second stream would fight it.
        if event.type() == QtCore.QEvent.KeyPress and not event.isAutoRepeat():
            key = event.key()
            if QtCore.Qt.Key_1 <= key <= QtCore.Qt.Key_6:
                self._pendant_axis = key - QtCore.Qt.Key_1 + 1
                self.statusbar.showMessage(
                    f"Pendant axis J{self._pendant_axis}", 2000)
                return True
            if key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down):
                axis = getattr(self, "_pendant_axis", 1)
                self.jog_press("joint", axis, +1 if key == QtCore.Qt.Key_Up else -1)
                return True

        if event.type() == QtCore.QEvent.KeyRelease and not event.isAutoRepeat():
            if event.key() in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down):
                self.jog_release()
                return True

        return super().eventFilter(obj, event)

    def start_servo(self):
        self.jogServoButton.setEnabled(False)
        self.statusbar.showMessage("Starting servo...")
        self._run_worker(ServoWorker(self.ros_node), self._servo_finished)

    def _servo_finished(self, ok, message):
        self.jogServoButton.setEnabled(True)
        if ok:
            self.statusbar.showMessage(message or "Servo started", 3000)
        else:
            self.statusbar.showMessage(f"Servo start failed: {message}", 6000)
            QtWidgets.QMessageBox.warning(self, "Servo start failed", message)

    def open_ws_calibration(self):
        dialog = WsCalibrationDialog(
            self.ros_node, self._current_position_values, self)
        dialog.exec_()

    # ── navigation ────────────────────────────────────────────────────────
    def _setup_nav(self):
        self.nav_buttons = [self.navPointPlanning, self.navPathPlanning, self.navIoControl, self.navMainTree, self.navClientControl, self.navErrorHandling]
        group = QtWidgets.QButtonGroup(self)
        group.setExclusive(True)
        for btn in self.nav_buttons:
            group.addButton(btn)

    # ── status LEDs ───────────────────────────────────────────────────────
    def _setup_leds(self):
        self.leds = {}
        for prefix in ("op", "fault", "homing", "emergency"):
            self.leds[prefix] = [getattr(self, f"{prefix}Led{i}") for i in range(1, 7)]

        # Everything starts dark; the first status message lights it up. No
        # demo values, so a dead topic looks dead instead of looking healthy.
        for prefix in self.leds:
            self.set_led_row(prefix, [False] * 6)

        self.robot_status = {
            "op": [False] * 6,
            "fault": [False] * 6,
            "homing": [False] * 6,
            "emergency": [False] * 6,
            "drive_mode": None,
        }
        self._home_in_flight = False
        self._action_busy = False

        self.homeButton.clicked.connect(self.do_home)
        self.resetButton.clicked.connect(self.do_reset)
        self.motionResetButton.clicked.connect(self.do_motion_reset)
        self.emergencyStopButton.clicked.connect(self.do_emergency_stop)

    def set_led_row(self, prefix, states, on_state="on"):
        for led, active in zip(self.leds[prefix], states):
            led.setProperty("state", on_state if active else "")
            restyle(led)

    def _refresh_status(self):
        """Pull the latest status from the ROS node and repaint the LEDs.

        Called from the existing 50ms GUI timer. Rows are only re-polished
        when they actually change — unpolish/polish on 24 widgets at 20Hz is
        wasteful and makes the whole UI feel sluggish.
        """
        status = self.ros_node.get_status()

        for prefix, on_state in (("op", "on"), ("fault", "fault"),
                                 ("homing", "on"), ("emergency", "fault")):
            if status[prefix] != self.robot_status[prefix]:
                self.set_led_row(prefix, status[prefix], on_state)
                self.robot_status[prefix] = status[prefix]

        self.robot_status["drive_mode"] = status["drive_mode"]
        self._update_button_states()

    # ── derived status ───────────────────────────────────────────────────
    @property
    def all_operational(self):
        return all(self.robot_status["op"])

    @property
    def any_emergency(self):
        return any(self.robot_status["emergency"])

    @property
    def any_fault(self):
        return any(self.robot_status["fault"])

    def _update_button_states(self):
        """HOME needs every joint operational and no emergency; RESET only
        needs no emergency. Both are blocked while another action runs."""
        home_ok = (self.all_operational
                   and not self.any_emergency
                   and not self._home_in_flight
                   and not self._action_busy)
        self.homeButton.setEnabled(home_ok)

        reset_ok = not self.any_emergency and not self._action_busy
        self.resetButton.setEnabled(reset_ok)
        self.motionResetButton.setEnabled(reset_ok)
        # EMERGENCY STOP is never disabled. It must work regardless of state.

    # ── sidebar actions ──────────────────────────────────────────────────
    def do_home(self):
        if self._home_in_flight or self._action_busy:
            return
        if not self.all_operational:
            QtWidgets.QMessageBox.warning(
                self, "Cannot HOME", "Not all joints are operational.")
            return
        if self.any_emergency:
            QtWidgets.QMessageBox.warning(
                self, "Cannot HOME", "Emergency condition detected.")
            return

        self._home_in_flight = True
        self._action_busy = True
        self._update_button_states()
        self.statusbar.showMessage("Homing...")

        worker = HomeWorker(self.ros_node)
        worker.progress.connect(self._action_progress)
        self._run_worker(worker, self._home_finished)

        # Safety re-enable: if the worker somehow never reports, the button
        # must not stay dead forever.
        self._home_guard = QtCore.QTimer(self)
        self._home_guard.setSingleShot(True)
        self._home_guard.timeout.connect(self._home_timed_out)
        self._home_guard.start(15000)

    def _home_timed_out(self):
        if self._home_in_flight:
            self._home_in_flight = False
            self._action_busy = False
            self._update_button_states()
            self.statusbar.showMessage("Home: no result received", 6000)

    def _home_finished(self, ok, message):
        if hasattr(self, "_home_guard"):
            self._home_guard.stop()
        self._home_in_flight = False
        self._action_busy = False
        self._update_button_states()
        if ok:
            self.statusbar.showMessage(message, 5000)
            # Home runs at SafeAuto thresholds; reflect that in the mode row.
            self.current_mode = "safeauto"
            self._set_mode_buttons("safeauto")
        else:
            self.statusbar.showMessage(f"Home failed: {message}", 8000)
            QtWidgets.QMessageBox.warning(self, "Home failed", message)

    def do_reset(self):
        if self._action_busy:
            return
        if self.any_emergency:
            QtWidgets.QMessageBox.warning(
                self, "Cannot RESET", "Emergency condition detected.")
            return

        self._action_busy = True
        self._update_button_states()
        self.statusbar.showMessage("Resetting...")

        worker = ResetWorker(self.ros_node)
        worker.progress.connect(self._action_progress)
        self._run_worker(worker, self._reset_finished)

    def _reset_finished(self, ok, message):
        self._action_busy = False
        self._update_button_states()
        if ok:
            self.statusbar.showMessage(message, 5000)
            # Reset leaves the drive in mode 9 — that is Jog.
            self.current_mode = "jog"
            self._set_mode_buttons("jog")
        else:
            self.statusbar.showMessage(f"Reset failed: {message}", 8000)
            QtWidgets.QMessageBox.warning(self, "Reset failed", message)

    def do_motion_reset(self):
        """Abort the running trajectory only.

        Deliberately does NOT reset faults or touch drive mode — in the web UI
        this button just calls stopMoveit(). It is the soft stop.
        """
        if self.any_emergency:
            QtWidgets.QMessageBox.warning(
                self, "Cannot RESET", "Emergency condition detected.")
            return
        self.ros_node.stop_moveit()
        self.statusbar.showMessage("Motion reset — trajectory aborted", 3000)
        self._cooldown(self.motionResetButton)

    def do_emergency_stop(self):
        """No guards and no confirmation dialog — an e-stop that asks
        'are you sure?' is not an e-stop."""
        if not self.ros_node.trigger_emergency():
            QtWidgets.QMessageBox.critical(
                self, "EMERGENCY STOP FAILED",
                "nextup_joint_interfaces is not installed, so the emergency "
                "trigger could not be published.\n\n"
                "Use the physical e-stop.")
            return
        self.statusbar.showMessage("EMERGENCY TRIGGERED", 10000)

    def _action_progress(self, message, percent):
        self.statusbar.showMessage(f"{percent}% — {message}")

    def _cooldown(self, button, ms=2000):
        """Briefly disable a button so it cannot be spammed."""
        button.setEnabled(False)
        QtCore.QTimer.singleShot(
            ms, lambda: (button.setEnabled(True), self._update_button_states()))

    # ── frames ────────────────────────────────────────────────────────────
    def _setup_frames(self):
        """Populate the servo frame selector.

        Valid options are "end" plus the name of every point with is_tf true —
        tf_loader_node publishes exactly those as TF frames (child of
        base_link, named after the point). Anything else would fail the
        lookupTransform in ui_command_node and silently fall back to "end".
        """
        self.current_frame = load_saved_frame()
        self._refresh_servo_frames()
        self.servoFrameSelect.currentIndexChanged.connect(self._on_frame_changed)

    def _refresh_servo_frames(self):
        """Rebuild the frame list: special frames, then TF points.

        The section headers are inserted as disabled items so the combo reads
        like the web UI's grouped menu. Only points with is_tf true appear
        under TF POINTS — tf_loader_node publishes exactly those as TF frames,
        so anything else would fail lookupTransform in ui_command_node.
        """
        model_items = []
        for label, value in SPECIAL_FRAMES:
            model_items.append((label, value))

        tf_points = [str(p.get("name", "")) for p in self.points
                     if p.get("is_tf") and p.get("name")]

        self.servoFrameSelect.blockSignals(True)
        self.servoFrameSelect.clear()

        self._add_combo_header("SPECIAL FRAMES")
        for label, value in model_items:
            self.servoFrameSelect.addItem(label, value)

        self._add_combo_header("TF POINTS")
        if tf_points:
            for name in tf_points:
                self.servoFrameSelect.addItem(name, name)
        else:
            self._add_combo_header("No TF points", italic=True)

        valid = [v for _, v in model_items] + tf_points
        if self.current_frame not in valid:
            # The saved frame is gone (point deleted or is_tf turned off).
            # Fall back rather than leaving the UI pointing at a frame the
            # robot cannot resolve.
            self.current_frame = DEFAULT_FRAME

        index = self.servoFrameSelect.findData(self.current_frame)
        if index >= 0:
            self.servoFrameSelect.setCurrentIndex(index)

        self.servoFrameSelect.blockSignals(False)
        self.activeFrameBadge.setText(f"ACTIVE  {self.current_frame}")

    def _add_combo_header(self, text, italic=False):
        """A non-selectable label row inside the combo."""
        combo = self.servoFrameSelect
        combo.addItem(text)
        index = combo.count() - 1
        item = combo.model().item(index)
        item.setEnabled(False)
        font = item.font()
        font.setBold(not italic)
        font.setItalic(italic)
        font.setPointSize(max(8, font.pointSize() - 1))
        item.setFont(font)

    def _on_frame_changed(self, _index):
        frame = self.servoFrameSelect.currentData()
        if not frame:
            return
        self.current_frame = frame
        self.activeFrameBadge.setText(f"ACTIVE  {frame}")
        self.ros_node.publish_frame_mode(frame)
        save_servo_frame(frame)
        self.statusbar.showMessage(f"Servo frame: {frame}", 2500)


    # ── jog ────────────────────────────────────────────────────────────────
    def _setup_jog(self):
        self.jog_buttons = {}
        for prefix, axes in (("joint", JOINT_AXES), ("cart", CART_AXES)):
            for i, axis in enumerate(axes, start=1):
                plus = getattr(self, f"{prefix}Plus{i}")
                minus = getattr(self, f"{prefix}Minus{i}")
                for btn, sign in ((plus, +1), (minus, -1)):
                    # pressed/released only. A click also emits `clicked`, so
                    # connecting that too would send a duplicate command.
                    btn.pressed.connect(
                        lambda p=prefix, a=i, s=sign: self.jog_press(p, a, s))
                    btn.released.connect(self.jog_release)
                self.jog_buttons[(prefix, i)] = (plus, minus)

        self._jog_cmd = None
        self._jog_timer = QtCore.QTimer(self)
        self._jog_timer.setInterval(JOG_REPEAT_MS)
        self._jog_timer.timeout.connect(self._jog_repeat)

    def _jog_repeat(self):
        if self._jog_cmd:
            self.ros_node.send(self._jog_cmd)

    def jog_press(self, mode, axis_index, sign):
        self._jog_cmd = self.ros_node.build_command(mode, axis_index, sign)
        self.ros_node.send(self._jog_cmd)
        self._jog_timer.start()
        self.statusbar.showMessage(f"Jog {self._jog_cmd}", 1000)

    def jog_release(self):
        self._jog_timer.stop()
        if self._jog_cmd:
            # Send the stop exactly once; the node decelerates internally.
            self.ros_node.send("0" + self._jog_cmd[1:])
            self.statusbar.showMessage("Jog stop", 1000)
            self._jog_cmd = None

    # ── clock ──────────────────────────────────────────────────────────────
    def _setup_clock(self):
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)
        self._tick()

    def _tick(self):
        now = QtCore.QTime.currentTime().toString("HH:mm:ss")
        try:
            load = os.getloadavg()
            load_text = f"{load[0]:.2f} {load[1]:.2f} {load[2]:.2f}"
        except OSError:
            load_text = "-- -- --"
        self.systemStatusLabel.setText(f"TIME : {now}\nUpTime: --\nLoad: {load_text}")

    def closeEvent(self, event):
        # Stop the arm before anything else — if a jog button is still held,
        # the drive keeps its last velocity once we stop publishing.
        try:
            self.jog_release()
        except Exception:
            pass
        # The viewer is a child process and would outlive us otherwise.
        self.stop_ws_sim()
        if self.ros_node:
            self.ros_node.destroy_node()
        super().closeEvent(event)


def main():
    rclpy.init()
    ros_node = ROSNode()
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)

    with open(QSS_FILE, "r", encoding="utf-8") as fh:
        app.setStyleSheet(fh.read())

    window = MainWindow(ros_node)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
