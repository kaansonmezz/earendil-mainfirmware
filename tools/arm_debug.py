# Requires Python 3. Run with the interpreter configured in .vscode/settings.json.
"""
Bench test GUI for the STM32F401 manipulation/drill firmware.

This tool does not modify or depend on the firmware source. It only sends the
existing text commands over the F401 UART link and prints every reply.

Dependencies:
  python -m pip install pyserial

Typical use:
  python tools/f401_test_gui.py --port COM7

If this Python has no tkinter, the tool automatically starts a browser GUI.
You can force that mode with:
  python tools/f401_test_gui.py --web --port COM7 --auto-connect

Dry-run UI test without hardware:
  python tools/f401_test_gui.py --dry-run
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
import webbrowser

from python_deps import ensure_pyserial_available

try:
    import tkinter as tk
    from tkinter import messagebox, scrolledtext, ttk
except ImportError:
    tk = None  # type: ignore[assignment]
    messagebox = None  # type: ignore[assignment]
    scrolledtext = None  # type: ignore[assignment]
    ttk = None  # type: ignore[assignment]
from typing import Callable, Deque, Dict, Iterable, List, Optional


ensure_pyserial_available()


DEFAULT_BAUD = 115200
DEFAULT_WEB_HOST = "127.0.0.1"
DEFAULT_WEB_PORT = 8765
MAX_SERIAL_LINE_BYTES = 4096
SERIAL_READ_CHUNK_BYTES = 4096
MAX_DESKTOP_LOG_LINES = 1500
DESKTOP_LOG_LINES_AFTER_TRIM = 1000
AXIS_NAMES = [
    "J1 Base",
    "J2 Shoulder",
    "J3 Elbow",
    "J4 Pitch",
    "J5 Twist",
    "J6 Gripper",
]
POSITION_AXES = (1, 2, 3)
STOP_MODES = ("coast", "brake", "hold", "hybrid")
STATUS_FIELDS = ("PWM", "DIR", "EN", "ANGLE", "VEL_DPS")


def blank_status_axes() -> List[Dict[str, str]]:
    return [{field: "-" for field in STATUS_FIELDS} for _ in AXIS_NAMES]


def blank_status_snapshot() -> Dict[str, object]:
    return {"mode": "-", "stopmode": "-", "axes": blank_status_axes()}


def parse_status_line(line: str) -> Optional[Dict[str, object]]:
    if not line.startswith("MODE:"):
        return None
    snapshot = blank_status_snapshot()
    axes = snapshot["axes"]
    assert isinstance(axes, list)
    for token in line.split():
        key, separator, value = token.partition(":")
        if not separator:
            continue
        key = key.upper()
        if key == "MODE":
            snapshot["mode"] = value
            continue
        if key == "STOPMODE":
            snapshot["stopmode"] = value
            continue
        if len(key) < 4 or key[0] != "J" or not key[1].isdigit() or key[2] != "_":
            continue
        axis_index = int(key[1]) - 1
        field = key[3:]
        if 0 <= axis_index < len(axes) and field in STATUS_FIELDS:
            axes[axis_index][field] = value
    if snapshot["mode"] not in ("safe", "arm", "drill"):
        return None
    if snapshot["stopmode"] not in STOP_MODES:
        return None
    for axis in axes:
        if any(axis[field] == "-" for field in STATUS_FIELDS):
            return None
        try:
            pwm = int(axis["PWM"])
            enabled = int(axis["EN"])
        except ValueError:
            return None
        if not 0 <= pwm <= 255 or enabled not in (0, 1):
            return None
        if axis["DIR"] not in ("stop", "forward", "backward"):
            return None
        for field in ("ANGLE", "VEL_DPS"):
            if axis[field] == "NA":
                continue
            try:
                value = float(axis[field])
            except ValueError:
                return None
            if not math.isfinite(value):
                return None
    return snapshot


@dataclass
class PendingSlider:
    value: int = 0
    sent_value: int = 0
    sent_at: float = 0.0


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


class SerialLink:
    def __init__(self, log: Callable[[str], None]) -> None:
        self.log = log
        self.serial = None
        self.dry_run = False
        self._lock = threading.RLock()
        self._rx_buffer = bytearray()
        self._rx_lines: Deque[str] = deque()
        self._discarding_oversize_line = False

    def is_open(self) -> bool:
        with self._lock:
            return self.dry_run or self.serial is not None

    def connect(self, port: str, baud: int, dry_run: bool) -> None:
        with self._lock:
            self._close_locked()
            if dry_run:
                self.dry_run = True
            else:
                if not port:
                    raise RuntimeError("Select a serial port first.")
                try:
                    import serial  # type: ignore
                except ImportError as exc:
                    raise RuntimeError("pyserial missing. Install with: python -m pip install pyserial") from exc
                self.serial = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0.2)
                time.sleep(0.2)
        if dry_run:
            self.log("LOCAL: dry-run serial link ready")
        else:
            self.log(f"LOCAL: connected to {port} @ {baud}")

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            finally:
                self.serial = None
        self.dry_run = False
        self._rx_buffer.clear()
        self._rx_lines.clear()
        self._discarding_oversize_line = False

    def send(self, line: str, *, log_tx: bool = True) -> None:
        line = line.strip()
        if not line:
            return
        if log_tx:
            self.log(f"TX: {line}")
        error: Optional[str] = None
        with self._lock:
            if self.dry_run:
                return
            if self.serial is None:
                error = "LOCAL: not connected"
            else:
                try:
                    self.serial.write((line + "\n").encode("ascii", errors="ignore"))
                except Exception as exc:  # pragma: no cover - depends on serial hardware
                    error = f"LOCAL: serial write failed, connection closed: {exc}"
                    self._close_locked()
        if error is not None:
            self.log(error)

    def read_lines(self, max_lines: int = 100) -> List[str]:
        if max_lines <= 0:
            return []
        error: Optional[str] = None
        with self._lock:
            if self.dry_run or self.serial is None:
                return []
            try:
                waiting = getattr(self.serial, "in_waiting", 0)
                if waiting > 0:
                    raw = self.serial.read(min(int(waiting), SERIAL_READ_CHUNK_BYTES))
                    self._ingest_bytes_locked(raw)
            except Exception as exc:  # pragma: no cover - depends on serial hardware
                error = f"LOCAL: serial read failed, connection closed: {exc}"
                self._close_locked()

            lines: List[str] = []
            while self._rx_lines and len(lines) < max_lines:
                lines.append(self._rx_lines.popleft())
        if error is not None:
            lines.append(error)
        return lines

    def _ingest_bytes_locked(self, raw: bytes) -> None:
        """Keep incomplete serial data buffered until a real line ending arrives."""
        for byte in raw:
            if self._discarding_oversize_line:
                if byte in (10, 13):
                    self._discarding_oversize_line = False
                continue
            if byte in (10, 13):
                if self._rx_buffer:
                    line = self._rx_buffer.decode("utf-8", errors="replace").strip()
                    self._rx_buffer.clear()
                    if line:
                        self._rx_lines.append(line)
                continue
            self._rx_buffer.append(byte)
            if len(self._rx_buffer) > MAX_SERIAL_LINE_BYTES:
                self._rx_buffer.clear()
                self._discarding_oversize_line = True
                self._rx_lines.append(
                    f"LOCAL: discarded serial line longer than {MAX_SERIAL_LINE_BYTES} bytes"
                )


class F401TestGui:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.root.title("F401 Manipulation/Drill Test GUI")
        self.root.geometry("1280x820")
        self.root.minsize(1100, 700)
        self.args = args
        self.link = SerialLink(self.log)
        self._log_line_count = 0

        self.port_var = tk.StringVar(value=args.port or "")
        self.baud_var = tk.IntVar(value=args.baud)
        self.dry_run_var = tk.BooleanVar(value=args.dry_run)
        self.auto_heartbeat_var = tk.BooleanVar(value=True)
        self.live_slider_var = tk.BooleanVar(value=False)
        self.connected_var = tk.StringVar(value="Disconnected")
        self.last_mode_var = tk.StringVar(value="Mode: unknown")
        self.last_fault_var = tk.StringVar(value="Fault: unknown")
        self.live_status_mode_var = tk.StringVar(value="Mode: -")
        self.live_status_stopmode_var = tk.StringVar(value="Stopmode: -")
        self.live_status_seen_var = tk.StringVar(value="Live status: waiting")
        self.live_status_vars = [
            {field: tk.StringVar(value="-") for field in STATUS_FIELDS}
            for _ in AXIS_NAMES
        ]

        self.axis_pwm_vars = [tk.IntVar(value=80) for _ in range(6)]
        self.axis_slider_vars = [tk.IntVar(value=0) for _ in range(6)]
        self.axis_slider_state = [PendingSlider() for _ in range(6)]

        self.position_angle_vars = [tk.StringVar(value="0") for _ in range(3)]
        self.rotate_angle_vars = [tk.StringVar(value="10") for _ in range(3)]
        self.stopmode_vars = [tk.StringVar(value="hold") for _ in range(3)]
        self.kp_vars = [tk.IntVar(value=1200) for _ in range(3)]
        self.kd_vars = [tk.IntVar(value=250) for _ in range(3)]
        self.minpwm_vars = [tk.IntVar(value=15) for _ in range(3)]
        self.tolerance_vars = [tk.StringVar(value="0.8") for _ in range(3)]
        self.as5600_vars = [tk.IntVar(value=i) for i in range(3)]

        self.tune_axis_var = tk.IntVar(value=1)
        self.tune_invert_var = tk.IntVar(value=0)
        self.tune_maxpwm_var = tk.IntVar(value=255)
        self.tune_default_var = tk.IntVar(value=100)
        self.tune_posgain_var = tk.IntVar(value=1000)
        self.tune_neggain_var = tk.IntVar(value=1000)
        self.tune_dead_var = tk.IntVar(value=50)
        self.tune_keyfwd_var = tk.IntVar(value=-1)
        self.tune_keyback_var = tk.IntVar(value=-1)
        self.tune_limits_var = tk.IntVar(value=0)
        self.tune_minangle_var = tk.StringVar(value="0")
        self.tune_maxangle_var = tk.StringVar(value="360")
        self.tune_brakepoint_var = tk.StringVar(value="0")

        self.elv_pwm_var = tk.IntVar(value=80)
        self.drill_pwm_var = tk.IntVar(value=80)
        self.elv_default_var = tk.IntVar(value=100)
        self.elv_max_var = tk.IntVar(value=255)
        self.elv_invert_var = tk.IntVar(value=0)
        self.drill_default_var = tk.IntVar(value=100)
        self.drill_max_var = tk.IntVar(value=255)
        self.drill_invert_var = tk.IntVar(value=0)

        self.joy_axis_var = tk.IntVar(value=1)
        self.joy_value_var = tk.IntVar(value=0)
        self.joy_button_id_var = tk.IntVar(value=1000)

        self.stream_channel_var = tk.IntVar(value=0)
        self.stream_interval_var = tk.IntVar(value=250)
        self.as5600_channel_var = tk.IntVar(value=0)

        self._build_ui()
        self.refresh_ports()
        self.root.after(30, self.poll_serial)
        self.root.after(500, self.heartbeat_tick)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        if args.auto_connect or args.dry_run:
            self.root.after(100, self.connect)

    def _build_ui(self) -> None:
        self._configure_style()
        self._build_top_bar()
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._build_console_tab(notebook)
        self._build_arm_tab(notebook)
        self._build_position_tab(notebook)
        self._build_tuning_tab(notebook)
        self._build_drill_tab(notebook)
        self._build_joy_tab(notebook)

    def _configure_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Danger.TButton", foreground="white", background="#b00020")
        style.map("Danger.TButton", background=[("active", "#d00028")])
        style.configure("Mode.TButton", padding=(10, 5))

    def _build_top_bar(self) -> None:
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill="x")

        ttk.Label(frame, text="Port").pack(side="left")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=12)
        self.port_combo.pack(side="left", padx=(4, 6))
        ttk.Button(frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(0, 12))

        ttk.Label(frame, text="Baud").pack(side="left")
        ttk.Entry(frame, textvariable=self.baud_var, width=8).pack(side="left", padx=(4, 8))
        ttk.Checkbutton(frame, text="Dry run", variable=self.dry_run_var).pack(side="left", padx=(0, 8))
        ttk.Button(frame, text="Connect", command=self.connect).pack(side="left", padx=(0, 4))
        ttk.Button(frame, text="Disconnect", command=self.disconnect).pack(side="left", padx=(0, 12))

        ttk.Label(frame, textvariable=self.connected_var).pack(side="left", padx=(0, 14))
        ttk.Label(frame, textvariable=self.last_mode_var).pack(side="left", padx=(0, 14))
        ttk.Label(frame, textvariable=self.last_fault_var).pack(side="left", padx=(0, 14))

        ttk.Checkbutton(frame, text="Auto heartbeat", variable=self.auto_heartbeat_var).pack(side="right")
        ttk.Button(frame, text="STOP ALL", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="right", padx=(8, 0)
        )

    def _build_console_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Console")

        quick = ttk.Frame(tab)
        quick.pack(fill="x")
        self._button_row(
            quick,
            [
                ("SAFE", "mode safe"),
                ("ARM", "mode arm confirm"),
                ("DRILL", "mode drill confirm"),
                ("Heartbeat", "heartbeat"),
                ("HB off", "set heartbeat 0"),
                ("HB 1000", "set heartbeat 1000"),
                ("Stop", "stop"),
                ("StopAll", "stopall"),
            ],
        )
        self._button_row(
            quick,
            [
                ("Help", "help"),
                ("Params", "params"),
                ("Get mode", "get mode"),
                ("Get fault", "get fault"),
                ("Get motors", "get motors"),
                ("Get sensors", "get sensors"),
                ("Get AS5600", "get as5600"),
                ("Stream off", "stream off"),
            ],
        )

        save_frame = ttk.Frame(tab)
        save_frame.pack(fill="x", pady=(6, 6))
        ttk.Button(save_frame, text="Save", command=lambda: self.send("save")).pack(side="left", padx=(0, 4))
        ttk.Button(save_frame, text="Safe + StopAll + Save", command=self.safe_stop_save).pack(side="left", padx=4)
        ttk.Button(save_frame, text="Factory reset", command=self.confirm_factory_reset).pack(side="left", padx=4)
        ttk.Button(save_frame, text="Quit/Exit firmware", command=lambda: self.send("quit")).pack(side="left", padx=4)

        self._build_live_status_panel(tab)

        seq_frame = ttk.LabelFrame(tab, text="seq wrapper")
        seq_frame.pack(fill="x", pady=(0, 8))
        self.seq_id_var = tk.IntVar(value=1)
        self.seq_command_var = tk.StringVar(value="params")
        ttk.Label(seq_frame, text="id").pack(side="left", padx=(6, 2))
        ttk.Spinbox(seq_frame, from_=0, to=999999, textvariable=self.seq_id_var, width=8).pack(side="left")
        ttk.Label(seq_frame, text="command").pack(side="left", padx=(8, 2))
        ttk.Entry(seq_frame, textvariable=self.seq_command_var).pack(side="left", fill="x", expand=True)
        ttk.Button(seq_frame, text="Send seq", command=self.send_seq).pack(side="left", padx=6)

        self.log_text = scrolledtext.ScrolledText(tab, height=24, wrap="word", state="disabled")
        self.log_text.pack(fill="both", expand=True, pady=(0, 6))

        entry_frame = ttk.Frame(tab)
        entry_frame.pack(fill="x")
        self.command_var = tk.StringVar()
        command_entry = ttk.Entry(entry_frame, textvariable=self.command_var)
        command_entry.pack(side="left", fill="x", expand=True)
        command_entry.bind("<Return>", lambda _event: self.send_manual())
        ttk.Button(entry_frame, text="Send", command=self.send_manual).pack(side="left", padx=(6, 0))
        ttk.Button(entry_frame, text="Clear log", command=self.clear_log).pack(side="left", padx=(6, 0))

    def _build_live_status_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Live F401 status")
        frame.pack(fill="x", pady=(0, 8))

        summary = ttk.Frame(frame)
        summary.pack(fill="x", padx=6, pady=(6, 2))
        ttk.Label(summary, textvariable=self.live_status_mode_var).pack(side="left", padx=(0, 16))
        ttk.Label(summary, textvariable=self.live_status_stopmode_var).pack(side="left", padx=(0, 16))
        ttk.Label(summary, textvariable=self.live_status_seen_var).pack(side="left")

        table = ttk.Frame(frame)
        table.pack(fill="x", padx=6, pady=(2, 6))
        headers = ("Joint", "PWM", "DIR", "EN", "Angle", "Vel dps")
        for column, text in enumerate(headers):
            ttk.Label(table, text=text).grid(row=0, column=column, sticky="w", padx=4, pady=2)
        for row, name in enumerate(AXIS_NAMES, start=1):
            ttk.Label(table, text=name).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            for column, field in enumerate(STATUS_FIELDS, start=1):
                ttk.Label(table, textvariable=self.live_status_vars[row - 1][field], width=12).grid(
                    row=row,
                    column=column,
                    sticky="w",
                    padx=4,
                    pady=2,
                )
        table.columnconfigure(0, weight=1)

    def _build_arm_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="ARM Drive")

        top = ttk.Frame(tab)
        top.pack(fill="x")
        ttk.Button(top, text="Mode ARM", command=lambda: self.send("mode arm confirm")).pack(side="left", padx=(0, 4))
        ttk.Button(top, text="Stop", command=lambda: self.send("stop")).pack(side="left", padx=4)
        ttk.Button(top, text="StopAll", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="left", padx=4
        )
        ttk.Checkbutton(top, text="Live axis sliders", variable=self.live_slider_var).pack(side="left", padx=12)

        grid = ttk.Frame(tab)
        grid.pack(fill="both", expand=True, pady=(8, 0))
        headers = ("Axis", "PWM", "Start", "Analog axis -1000..1000", "Send")
        for col, text in enumerate(headers):
            ttk.Label(grid, text=text).grid(row=0, column=col, sticky="w", padx=4, pady=4)
        grid.columnconfigure(3, weight=1)

        for i, name in enumerate(AXIS_NAMES, start=1):
            row = i
            ttk.Label(grid, text=name, width=13).grid(row=row, column=0, sticky="w", padx=4, pady=4)
            ttk.Spinbox(
                grid,
                from_=0,
                to=255,
                textvariable=self.axis_pwm_vars[i - 1],
                width=6,
            ).grid(row=row, column=1, sticky="w", padx=4)
            start_frame = ttk.Frame(grid)
            start_frame.grid(row=row, column=2, sticky="w", padx=4)
            ttk.Button(start_frame, text="Fwd", command=lambda axis=i: self.drive_axis(axis, "forward")).pack(
                side="left", padx=(0, 2)
            )
            ttk.Button(start_frame, text="Back", command=lambda axis=i: self.drive_axis(axis, "backward")).pack(
                side="left", padx=2
            )
            ttk.Button(start_frame, text="Stop", command=lambda axis=i: self.send(f"stop {axis}")).pack(
                side="left", padx=2
            )
            scale = ttk.Scale(
                grid,
                from_=-1000,
                to=1000,
                orient="horizontal",
                variable=self.axis_slider_vars[i - 1],
                command=lambda value, axis=i: self.on_axis_slider(axis, value),
            )
            scale.grid(row=row, column=3, sticky="ew", padx=4)
            scale.bind("<ButtonRelease-1>", lambda _event, axis=i: self.send_axis_slider(axis))
            send_frame = ttk.Frame(grid)
            send_frame.grid(row=row, column=4, sticky="w", padx=4)
            ttk.Button(send_frame, text="Send", command=lambda axis=i: self.send_axis_slider(axis)).pack(
                side="left", padx=(0, 2)
            )
            ttk.Button(send_frame, text="0+Stop", command=lambda axis=i: self.zero_axis_slider(axis)).pack(
                side="left", padx=2
            )

    def _build_position_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Position/PID")

        sensor_frame = ttk.LabelFrame(tab, text="Sensors and stream")
        sensor_frame.pack(fill="x")
        ttk.Button(sensor_frame, text="Get sensors", command=lambda: self.send("get sensors")).pack(
            side="left", padx=4, pady=6
        )
        ttk.Button(sensor_frame, text="Get all AS5600", command=lambda: self.send("get as5600")).pack(
            side="left", padx=4
        )
        ttk.Label(sensor_frame, text="CH").pack(side="left", padx=(16, 2))
        ttk.Spinbox(sensor_frame, from_=0, to=7, textvariable=self.as5600_channel_var, width=4).pack(side="left")
        ttk.Button(sensor_frame, text="Get CH", command=self.get_as5600_channel).pack(side="left", padx=4)
        ttk.Label(sensor_frame, text="Stream CH").pack(side="left", padx=(16, 2))
        ttk.Spinbox(sensor_frame, from_=0, to=7, textvariable=self.stream_channel_var, width=4).pack(side="left")
        ttk.Label(sensor_frame, text="ms").pack(side="left", padx=(8, 2))
        ttk.Spinbox(sensor_frame, from_=50, to=5000, textvariable=self.stream_interval_var, width=6).pack(side="left")
        ttk.Button(sensor_frame, text="Start stream", command=self.start_stream).pack(side="left", padx=4)
        ttk.Button(sensor_frame, text="Stream off", command=lambda: self.send("stream off")).pack(side="left", padx=4)

        rows = ttk.Frame(tab)
        rows.pack(fill="both", expand=True, pady=(8, 0))
        for index, axis in enumerate(POSITION_AXES):
            frame = ttk.LabelFrame(rows, text=AXIS_NAMES[axis - 1])
            frame.pack(fill="x", pady=4)
            self._build_position_axis_row(frame, index, axis)

    def _build_position_axis_row(self, frame: ttk.LabelFrame, index: int, axis: int) -> None:
        ttk.Button(frame, text="Zero", command=lambda: self.send(f"zero {axis}")).grid(
            row=0, column=0, padx=4, pady=4
        )
        ttk.Label(frame, text="Angle").grid(row=0, column=1, sticky="e")
        ttk.Entry(frame, textvariable=self.position_angle_vars[index], width=8).grid(row=0, column=2, padx=4)
        ttk.Button(frame, text="Goto", command=lambda: self.goto_axis(axis, index)).grid(row=0, column=3, padx=4)
        ttk.Label(frame, text="Rotate").grid(row=0, column=4, sticky="e")
        ttk.Entry(frame, textvariable=self.rotate_angle_vars[index], width=8).grid(row=0, column=5, padx=4)
        ttk.Button(frame, text="Move", command=lambda: self.rotate_axis(axis, index)).grid(row=0, column=6, padx=4)
        ttk.Button(frame, text="Stop", command=lambda: self.send(f"stop {axis}")).grid(row=0, column=7, padx=4)
        ttk.Button(frame, text="Hold now", command=lambda: self.hold_axis(axis)).grid(row=0, column=8, padx=4)

        ttk.Label(frame, text="Stopmode").grid(row=1, column=0, sticky="e", padx=4)
        ttk.Combobox(frame, textvariable=self.stopmode_vars[index], values=STOP_MODES, width=8, state="readonly").grid(
            row=1, column=1, padx=4
        )
        ttk.Button(frame, text="Set", command=lambda: self.set_stopmode(axis, index)).grid(row=1, column=2, padx=4)
        ttk.Label(frame, text="KP").grid(row=1, column=3, sticky="e")
        ttk.Spinbox(frame, from_=0, to=10000, textvariable=self.kp_vars[index], width=7).grid(row=1, column=4, padx=4)
        ttk.Label(frame, text="KD").grid(row=1, column=5, sticky="e")
        ttk.Spinbox(frame, from_=0, to=10000, textvariable=self.kd_vars[index], width=7).grid(row=1, column=6, padx=4)
        ttk.Button(frame, text="Set KP/KD", command=lambda: self.set_kp_kd(axis, index)).grid(
            row=1, column=7, padx=4
        )

        ttk.Label(frame, text="MinPWM").grid(row=2, column=0, sticky="e", padx=4)
        ttk.Spinbox(frame, from_=0, to=255, textvariable=self.minpwm_vars[index], width=7).grid(
            row=2, column=1, padx=4
        )
        ttk.Label(frame, text="Tol deg").grid(row=2, column=2, sticky="e")
        ttk.Entry(frame, textvariable=self.tolerance_vars[index], width=7).grid(row=2, column=3, padx=4)
        ttk.Label(frame, text="AS5600 CH").grid(row=2, column=4, sticky="e")
        ttk.Spinbox(frame, from_=-1, to=7, textvariable=self.as5600_vars[index], width=5).grid(
            row=2, column=5, padx=4
        )
        ttk.Button(frame, text="Set PID/sensor", command=lambda: self.set_pid_sensor(axis, index)).grid(
            row=2, column=6, padx=4
        )
        ttk.Button(frame, text="Active hold preset", command=lambda: self.active_hold_preset(axis, index)).grid(
            row=2, column=7, padx=4
        )

    def _build_tuning_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Tuning")

        select = ttk.Frame(tab)
        select.pack(fill="x")
        ttk.Label(select, text="Joint").pack(side="left")
        ttk.Spinbox(select, from_=1, to=6, textvariable=self.tune_axis_var, width=5).pack(side="left", padx=4)
        ttk.Button(select, text="Params", command=lambda: self.send("params")).pack(side="left", padx=4)
        ttk.Button(select, text="Get motors", command=lambda: self.send("get motors")).pack(side="left", padx=4)
        ttk.Button(select, text="Get fault", command=lambda: self.send("get fault")).pack(side="left", padx=4)

        general = ttk.LabelFrame(tab, text="General axis settings")
        general.pack(fill="x", pady=8)
        self._add_tune_scale(general, "invert", self.tune_invert_var, 0, 1, 0, "invert")
        self._add_tune_scale(general, "maxpwm", self.tune_maxpwm_var, 0, 255, 1, "maxpwm")
        self._add_tune_scale(general, "default", self.tune_default_var, 0, 255, 2, "default")
        self._add_tune_scale(general, "posgain", self.tune_posgain_var, 100, 3000, 3, "posgain")
        self._add_tune_scale(general, "neggain", self.tune_neggain_var, 100, 3000, 4, "neggain")
        self._add_tune_scale(general, "dead", self.tune_dead_var, 0, 400, 5, "dead")

        keys = ttk.LabelFrame(tab, text="Keybinds and position-only mapping")
        keys.pack(fill="x", pady=8)
        ttk.Label(keys, text="KEYFWD").grid(row=0, column=0, padx=4, pady=4)
        ttk.Spinbox(keys, from_=-1, to=32767, textvariable=self.tune_keyfwd_var, width=8).grid(row=0, column=1)
        ttk.Button(keys, text="Set keyfwd", command=lambda: self.set_tune_value("keybind", self.tune_keyfwd_var)).grid(
            row=0, column=2, padx=4
        )
        ttk.Label(keys, text="KEYBACK").grid(row=0, column=3, padx=4)
        ttk.Spinbox(keys, from_=-1, to=32767, textvariable=self.tune_keyback_var, width=8).grid(row=0, column=4)
        ttk.Button(
            keys,
            text="Set keyback",
            command=lambda: self.set_tune_value("negkeybind", self.tune_keyback_var),
        ).grid(row=0, column=5, padx=4)
        ttk.Label(keys, text="AS5600 CH J1-J3").grid(row=1, column=0, padx=4, pady=4)
        self.tune_as5600_var = tk.IntVar(value=0)
        ttk.Spinbox(keys, from_=-1, to=7, textvariable=self.tune_as5600_var, width=8).grid(row=1, column=1)
        ttk.Button(keys, text="Set as5600", command=lambda: self.set_tune_value("as5600", self.tune_as5600_var)).grid(
            row=1, column=2, padx=4
        )

        safety = ttk.LabelFrame(tab, text="J1-J3 limits and brakepoint")
        safety.pack(fill="x", pady=8)
        ttk.Label(safety, text="limits").grid(row=0, column=0, padx=4, pady=4)
        ttk.Spinbox(safety, from_=0, to=1, textvariable=self.tune_limits_var, width=5).grid(row=0, column=1)
        ttk.Button(safety, text="Set limits", command=lambda: self.set_tune_value("limits", self.tune_limits_var)).grid(
            row=0, column=2, padx=4
        )
        ttk.Label(safety, text="minangle").grid(row=0, column=3, padx=4)
        ttk.Entry(safety, textvariable=self.tune_minangle_var, width=8).grid(row=0, column=4)
        ttk.Button(
            safety,
            text="Set minangle",
            command=lambda: self.set_tune_text("minangle", self.tune_minangle_var),
        ).grid(row=0, column=5, padx=4)
        ttk.Label(safety, text="maxangle").grid(row=1, column=3, padx=4)
        ttk.Entry(safety, textvariable=self.tune_maxangle_var, width=8).grid(row=1, column=4)
        ttk.Button(
            safety,
            text="Set maxangle",
            command=lambda: self.set_tune_text("maxangle", self.tune_maxangle_var),
        ).grid(row=1, column=5, padx=4)
        ttk.Label(safety, text="brakepoint").grid(row=1, column=0, padx=4, pady=4)
        ttk.Entry(safety, textvariable=self.tune_brakepoint_var, width=8).grid(row=1, column=1)
        ttk.Button(
            safety,
            text="Set brakepoint",
            command=lambda: self.set_tune_text("brakepoint", self.tune_brakepoint_var),
        ).grid(row=1, column=2, padx=4)
        ttk.Button(safety, text="Brakepoint off", command=lambda: self.send_tune_command("brakepoint off")).grid(
            row=1, column=6, padx=4
        )

    def _build_drill_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="DRILL")

        top = ttk.Frame(tab)
        top.pack(fill="x")
        ttk.Button(top, text="Mode DRILL", command=lambda: self.send("mode drill confirm")).pack(side="left", padx=4)
        ttk.Button(top, text="Mode SAFE", command=lambda: self.send("mode safe")).pack(side="left", padx=4)
        ttk.Button(top, text="StopAll", style="Danger.TButton", command=lambda: self.send("stopall")).pack(
            side="left", padx=4
        )

        elv = ttk.LabelFrame(tab, text="Elevator: BTS4 + BTS5")
        elv.pack(fill="x", pady=8)
        self._build_drill_pwm_row(
            elv,
            self.elv_pwm_var,
            [("Up", lambda: self.send(f"elv up {self.elv_pwm_var.get()}")),
             ("Down", lambda: self.send(f"elv down {self.elv_pwm_var.get()}")),
             ("Stop", lambda: self.send("elv stop"))],
        )
        self._build_drill_settings_row(
            elv,
            "elv",
            self.elv_default_var,
            self.elv_max_var,
            self.elv_invert_var,
            row=1,
        )

        drill = ttk.LabelFrame(tab, text="Drill: BTS6")
        drill.pack(fill="x", pady=8)
        self._build_drill_pwm_row(
            drill,
            self.drill_pwm_var,
            [("Dig", lambda: self.send(f"drill dig {self.drill_pwm_var.get()}")),
             ("Extract", lambda: self.send(f"drill extract {self.drill_pwm_var.get()}")),
             ("Stop", lambda: self.send("drill stop"))],
        )
        self._build_drill_settings_row(
            drill,
            "drill",
            self.drill_default_var,
            self.drill_max_var,
            self.drill_invert_var,
            row=1,
        )

    def _build_joy_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Joy/Keybind Test")

        axis_frame = ttk.LabelFrame(tab, text="joy axis")
        axis_frame.pack(fill="x")
        ttk.Label(axis_frame, text="Joint").pack(side="left", padx=4, pady=6)
        ttk.Spinbox(axis_frame, from_=1, to=6, textvariable=self.joy_axis_var, width=5).pack(side="left")
        ttk.Scale(axis_frame, from_=-1000, to=1000, variable=self.joy_value_var, orient="horizontal").pack(
            side="left", fill="x", expand=True, padx=8
        )
        ttk.Button(axis_frame, text="Send joy axis", command=self.send_joy_axis).pack(side="left", padx=4)
        ttk.Button(axis_frame, text="Zero + send", command=self.zero_joy_axis).pack(side="left", padx=4)

        button_frame = ttk.LabelFrame(tab, text="joy button")
        button_frame.pack(fill="x", pady=8)
        ttk.Label(button_frame, text="Button id").pack(side="left", padx=4, pady=6)
        ttk.Spinbox(button_frame, from_=0, to=32767, textvariable=self.joy_button_id_var, width=8).pack(side="left")
        ttk.Button(button_frame, text="Press", command=lambda: self.send_joy_button(1)).pack(side="left", padx=4)
        ttk.Button(button_frame, text="Release", command=lambda: self.send_joy_button(0)).pack(side="left", padx=4)

        preset_frame = ttk.LabelFrame(tab, text="D-pad synthetic ids")
        preset_frame.pack(fill="x", pady=8)
        for label, button_id in (("Hat up 1000", 1000), ("Hat down 1001", 1001), ("Hat left 1002", 1002), ("Hat right 1003", 1003)):
            ttk.Button(preset_frame, text=f"{label} press", command=lambda bid=button_id: self.send(f"joy button {bid} 1")).pack(
                side="left", padx=4, pady=6
            )
            ttk.Button(preset_frame, text="release", command=lambda bid=button_id: self.send(f"joy button {bid} 0")).pack(
                side="left", padx=(0, 8), pady=6
            )

    def _build_drill_pwm_row(self, frame: ttk.LabelFrame, pwm_var: tk.IntVar, buttons: Iterable[tuple[str, Callable[[], None]]]) -> None:
        ttk.Label(frame, text="PWM").grid(row=0, column=0, padx=4, pady=6)
        ttk.Scale(frame, from_=0, to=255, variable=pwm_var, orient="horizontal").grid(
            row=0, column=1, sticky="ew", padx=4
        )
        ttk.Spinbox(frame, from_=0, to=255, textvariable=pwm_var, width=6).grid(row=0, column=2, padx=4)
        for col, (text, command) in enumerate(buttons, start=3):
            ttk.Button(frame, text=text, command=command).grid(row=0, column=col, padx=4)
        frame.columnconfigure(1, weight=1)

    def _build_drill_settings_row(
        self,
        frame: ttk.LabelFrame,
        prefix: str,
        default_var: tk.IntVar,
        max_var: tk.IntVar,
        invert_var: tk.IntVar,
        row: int,
    ) -> None:
        ttk.Label(frame, text="default").grid(row=row, column=0, padx=4, pady=6)
        ttk.Spinbox(frame, from_=0, to=255, textvariable=default_var, width=6).grid(row=row, column=1)
        ttk.Button(frame, text="Set default", command=lambda: self.send(f"{prefix} default {default_var.get()}")).grid(
            row=row, column=2, padx=4
        )
        ttk.Label(frame, text="maxpwm").grid(row=row, column=3, padx=4)
        ttk.Spinbox(frame, from_=0, to=255, textvariable=max_var, width=6).grid(row=row, column=4)
        ttk.Button(frame, text="Set maxpwm", command=lambda: self.send(f"{prefix} maxpwm {max_var.get()}")).grid(
            row=row, column=5, padx=4
        )
        ttk.Label(frame, text="invert").grid(row=row, column=6, padx=4)
        ttk.Spinbox(frame, from_=0, to=1, textvariable=invert_var, width=4).grid(row=row, column=7)
        ttk.Button(frame, text="Set invert", command=lambda: self.send(f"{prefix} invert {invert_var.get()}")).grid(
            row=row, column=8, padx=4
        )

    def _button_row(self, parent: ttk.Frame, buttons: Iterable[tuple[str, str]]) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        for text, command in buttons:
            style = "Danger.TButton" if "Stop" in text or "STOP" in text else "TButton"
            ttk.Button(row, text=text, style=style, command=lambda cmd=command: self.send(cmd)).pack(
                side="left", padx=3
            )

    def _add_tune_scale(
        self,
        parent: ttk.LabelFrame,
        label: str,
        variable: tk.IntVar,
        low: int,
        high: int,
        row: int,
        command_name: str,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", padx=4, pady=4)
        ttk.Scale(parent, from_=low, to=high, variable=variable, orient="horizontal").grid(
            row=row, column=1, sticky="ew", padx=4
        )
        ttk.Spinbox(parent, from_=low, to=high, textvariable=variable, width=8).grid(row=row, column=2, padx=4)
        ttk.Button(parent, text="Set", command=lambda: self.set_tune_value(command_name, variable)).grid(
            row=row, column=3, padx=4
        )
        parent.columnconfigure(1, weight=1)

    def refresh_ports(self) -> None:
        ports = serial_ports()
        self.port_combo["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def connect(self) -> None:
        try:
            self.link.connect(self.port_var.get(), int(self.baud_var.get()), self.dry_run_var.get())
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            self.log(f"LOCAL: connect failed: {exc}")
            return
        self.connected_var.set("Connected" if not self.dry_run_var.get() else "Dry run")

    def disconnect(self) -> None:
        self.link.close()
        self.connected_var.set("Disconnected")
        self.log("LOCAL: disconnected")

    def log(self, text: str) -> None:
        if not hasattr(self, "log_text"):
            print(text, flush=True)
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self._log_line_count += text.count("\n") + 1
        if self._log_line_count > MAX_DESKTOP_LOG_LINES:
            lines_to_remove = self._log_line_count - DESKTOP_LOG_LINES_AFTER_TRIM
            self.log_text.delete("1.0", f"{lines_to_remove + 1}.0")
            self._log_line_count -= lines_to_remove
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self._log_line_count = 0

    def send(self, command: str) -> None:
        self.link.send(command)

    def send_many(self, commands: Iterable[str]) -> None:
        for command in commands:
            self.send(command)

    def send_manual(self) -> None:
        command = self.command_var.get().strip()
        if command:
            self.send(command)
            self.command_var.set("")

    def send_seq(self) -> None:
        self.send(f"seq {self.seq_id_var.get()} {self.seq_command_var.get().strip()}")
        self.seq_id_var.set(self.seq_id_var.get() + 1)

    def poll_serial(self) -> None:
        for line in self.link.read_lines():
            if not self.update_status_from_line(line):
                self.log(line if line.startswith("LOCAL:") else f"F401: {line}")
        if not self.link.is_open() and self.connected_var.get() not in ("Disconnected", "Dry run"):
            self.connected_var.set("Disconnected")
        self.root.after(30, self.poll_serial)

    def heartbeat_tick(self) -> None:
        if self.auto_heartbeat_var.get() and self.link.is_open():
            self.link.send("heartbeat", log_tx=False)
        self.root.after(500, self.heartbeat_tick)

    def update_status_from_line(self, line: str) -> bool:
        if line == "OK HEARTBEAT":
            return True
        status = parse_status_line(line)
        if status is not None:
            self.apply_live_status(status)
            return True
        if line.startswith("OK MODE "):
            self.last_mode_var.set("Mode: " + line.removeprefix("OK MODE ").strip())
        elif line.startswith("OK FAULT "):
            parts = line.split()
            if len(parts) >= 3:
                self.last_fault_var.set("Fault: " + " ".join(parts[2:5]))
        elif "HEARTBEAT_TIMEOUT" in line:
            self.last_fault_var.set("Fault: heartbeat timeout")
        return False

    def apply_live_status(self, status: Dict[str, object]) -> None:
        mode = str(status.get("mode", "-"))
        stopmode = str(status.get("stopmode", "-"))
        self.live_status_mode_var.set("Mode: " + mode)
        self.live_status_stopmode_var.set("Stopmode: " + stopmode)
        self.live_status_seen_var.set("Live status: " + time.strftime("%H:%M:%S"))
        self.last_mode_var.set("Mode: " + mode)
        axes = status.get("axes", [])
        if not isinstance(axes, list):
            return
        for index, axis in enumerate(axes[: len(self.live_status_vars)]):
            if not isinstance(axis, dict):
                continue
            for field in STATUS_FIELDS:
                self.live_status_vars[index][field].set(str(axis.get(field, "-")))

    def safe_stop_save(self) -> None:
        self.send_many(("stopall", "mode safe", "save"))

    def confirm_factory_reset(self) -> None:
        if messagebox.askyesno("Factory reset", "Reset RAM settings to firmware defaults?"):
            self.send("factoryreset confirm")

    def drive_axis(self, axis: int, direction: str) -> None:
        pwm = clamp_int(int(self.axis_pwm_vars[axis - 1].get()), 0, 255)
        self.send(f"{direction} {axis} {pwm}")

    def on_axis_slider(self, axis: int, value: str) -> None:
        index = axis - 1
        state = self.axis_slider_state[index]
        state.value = int(float(value))
        if not self.live_slider_var.get():
            return
        now = time.monotonic()
        if now - state.sent_at < 0.06 and abs(state.value - state.sent_value) < 40:
            return
        self.send_axis_slider(axis)

    def send_axis_slider(self, axis: int) -> None:
        index = axis - 1
        value = clamp_int(int(self.axis_slider_vars[index].get()), -1000, 1000)
        self.axis_slider_state[index].sent_value = value
        self.axis_slider_state[index].sent_at = time.monotonic()
        self.send(f"axis {axis} {value}")

    def zero_axis_slider(self, axis: int) -> None:
        self.axis_slider_vars[axis - 1].set(0)
        self.send(f"axis {axis} 0")
        self.send(f"stop {axis}")

    def goto_axis(self, axis: int, index: int) -> None:
        self.send(f"goto {axis} {self.position_angle_vars[index].get().strip()}")

    def rotate_axis(self, axis: int, index: int) -> None:
        self.send(f"rotate {axis} {self.rotate_angle_vars[index].get().strip()}")

    def hold_axis(self, axis: int) -> None:
        self.send_many((f"set {axis} stopmode hold", f"stop {axis}"))

    def set_stopmode(self, axis: int, index: int) -> None:
        self.send(f"set {axis} stopmode {self.stopmode_vars[index].get()}")

    def set_kp_kd(self, axis: int, index: int) -> None:
        self.send_many((f"set {axis} kp {self.kp_vars[index].get()}", f"set {axis} kd {self.kd_vars[index].get()}"))

    def set_pid_sensor(self, axis: int, index: int) -> None:
        self.send_many(
            (
                f"set {axis} minpwm {self.minpwm_vars[index].get()}",
                f"set {axis} tolerance {self.tolerance_vars[index].get().strip()}",
                f"set {axis} as5600 {self.as5600_vars[index].get()}",
            )
        )

    def active_hold_preset(self, axis: int, index: int) -> None:
        self.send_many(
            (
                "set heartbeat 0",
                "mode arm confirm",
                f"set {axis} as5600 {self.as5600_vars[index].get()}",
                f"set {axis} stopmode hold",
                f"set {axis} tolerance {self.tolerance_vars[index].get().strip()}",
                f"set {axis} kp {self.kp_vars[index].get()}",
                f"set {axis} kd {self.kd_vars[index].get()}",
                f"set {axis} minpwm {self.minpwm_vars[index].get()}",
                f"stop {axis}",
            )
        )

    def get_as5600_channel(self) -> None:
        self.send(f"get as5600 {self.as5600_channel_var.get()}")

    def start_stream(self) -> None:
        self.send(f"stream as5600 {self.stream_channel_var.get()} {self.stream_interval_var.get()}")

    def selected_tune_axis(self) -> int:
        return clamp_int(int(self.tune_axis_var.get()), 1, 6)

    def send_tune_command(self, tail: str) -> None:
        self.send(f"set {self.selected_tune_axis()} {tail}")

    def set_tune_value(self, command_name: str, variable: tk.IntVar) -> None:
        self.send_tune_command(f"{command_name} {int(variable.get())}")

    def set_tune_text(self, command_name: str, variable: tk.StringVar) -> None:
        self.send_tune_command(f"{command_name} {variable.get().strip()}")

    def send_joy_axis(self) -> None:
        self.send(f"joy axis {self.joy_axis_var.get()} {self.joy_value_var.get()}")

    def zero_joy_axis(self) -> None:
        self.joy_value_var.set(0)
        self.send_joy_axis()

    def send_joy_button(self, state: int) -> None:
        self.send(f"joy button {self.joy_button_id_var.get()} {state}")

    def on_close(self) -> None:
        if self.link.is_open():
            try:
                self.send("stopall")
            finally:
                self.link.close()
        self.root.destroy()


WEB_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>F401 Test GUI</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fa;
      --panel: #ffffff;
      --line: #d7dde5;
      --text: #17202a;
      --muted: #627386;
      --blue: #1b66c9;
      --red: #bd1e34;
      --green: #14805e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Segoe UI, Arial, sans-serif;
      font-size: 14px;
    }
    header {
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 10px 12px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
      position: sticky;
      top: 0;
      z-index: 3;
    }
    main { padding: 10px; }
    .status { margin-left: auto; color: var(--muted); display: flex; gap: 14px; }
    .tabs { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 10px; }
    .tab-btn {
      border: 1px solid var(--line);
      background: var(--panel);
      padding: 7px 10px;
      border-radius: 6px;
      cursor: pointer;
    }
    .tab-btn.active { border-color: var(--blue); color: var(--blue); font-weight: 600; }
    .tab { display: none; }
    .tab.active { display: block; }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      margin-bottom: 10px;
    }
    .grid { display: grid; gap: 8px; }
    .grid.cols2 { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .grid.cols3 { grid-template-columns: repeat(3, minmax(0, 1fr)); }
    .row { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; margin: 4px 0; }
    .axis-row {
      display: grid;
      grid-template-columns: 130px 70px 210px minmax(220px, 1fr) 145px;
      gap: 8px;
      align-items: center;
      padding: 6px 0;
      border-bottom: 1px solid #edf1f5;
    }
    .axis-row:last-child { border-bottom: 0; }
    button {
      border: 1px solid var(--line);
      background: #fff;
      color: var(--text);
      padding: 6px 9px;
      border-radius: 6px;
      cursor: pointer;
    }
    button:hover { border-color: var(--blue); }
    button.primary { background: var(--blue); color: #fff; border-color: var(--blue); }
    button.danger { background: var(--red); color: #fff; border-color: var(--red); }
    button.green { background: var(--green); color: #fff; border-color: var(--green); }
    input, select {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 6px 7px;
      min-height: 31px;
    }
    input[type=number] { width: 84px; }
    input[type=range] { width: 100%; }
    label { color: var(--muted); }
    .small { color: var(--muted); font-size: 12px; }
    .mono { font-family: Consolas, monospace; }
    h2 {
      font-size: 16px;
      margin: 0 0 8px;
    }
    #log {
      height: 330px;
      overflow: auto;
      background: #0f1720;
      color: #d7e7ff;
      border-radius: 8px;
      padding: 8px;
      font-family: Consolas, monospace;
      white-space: pre-wrap;
    }
    #cmd { flex: 1; min-width: 240px; }
    .status-table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 8px;
    }
    .status-table th,
    .status-table td {
      text-align: left;
      padding: 5px 6px;
      border-bottom: 1px solid #edf1f5;
      font-variant-numeric: tabular-nums;
    }
    .status-table th { color: var(--muted); font-weight: 600; }
    @media (max-width: 900px) {
      .axis-row { grid-template-columns: 1fr; }
      .grid.cols2, .grid.cols3 { grid-template-columns: 1fr; }
      header { align-items: stretch; flex-direction: column; }
      .status { margin-left: 0; }
    }
  </style>
</head>
<body>
  <header>
    <label>Port <select id="port"></select></label>
    <button onclick="refreshPorts()">Refresh</button>
    <label>Baud <input id="baud" type="number" value="115200"></label>
    <label><input id="dry" type="checkbox"> Dry run</label>
    <button class="primary" onclick="connect()">Connect</button>
    <button onclick="disconnect()">Disconnect</button>
    <button class="danger" onclick="send('stopall')">STOP ALL</button>
    <label><input id="autohb" type="checkbox" checked onchange="setHeartbeat()"> Auto heartbeat</label>
    <div class="status">
      <span id="connected">Disconnected</span>
      <span id="mode">Mode: unknown</span>
      <span id="fault">Fault: unknown</span>
    </div>
  </header>
  <main>
    <div class="tabs" id="tabs"></div>

    <section id="console" class="tab active">
      <div class="panel">
        <h2>Quick commands</h2>
        <div class="row">
          <button onclick="send('mode safe')">SAFE</button>
          <button onclick="send('mode arm confirm')">ARM</button>
          <button onclick="send('mode drill confirm')">DRILL</button>
          <button onclick="send('heartbeat')">Heartbeat</button>
          <button onclick="send('set heartbeat 0')">HB off</button>
          <button onclick="send('set heartbeat 1000')">HB 1000</button>
          <button onclick="send('stop')">Stop</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
        </div>
        <div class="row">
          <button onclick="send('help')">Help</button>
          <button onclick="send('params')">Params</button>
          <button onclick="send('get mode')">Get mode</button>
          <button onclick="send('get fault')">Get fault</button>
          <button onclick="send('get motors')">Get motors</button>
          <button onclick="send('get sensors')">Get sensors</button>
          <button onclick="send('get as5600')">Get AS5600</button>
          <button onclick="send('stream off')">Stream off</button>
        </div>
        <div class="row">
          <button onclick="send('save')">Save</button>
          <button onclick="many(['stopall','mode safe','save'])">Safe + StopAll + Save</button>
          <button onclick="factoryReset()">Factory reset</button>
          <button onclick="send('quit')">Quit firmware</button>
        </div>
      </div>
      <div class="panel">
        <h2>Live F401 status</h2>
        <div class="row small mono">
          <span id="liveMode">Mode: -</span>
          <span id="liveStopmode">Stopmode: -</span>
          <span id="liveSeen">Live status: waiting</span>
        </div>
        <table class="status-table">
          <thead>
            <tr><th>Joint</th><th>PWM</th><th>DIR</th><th>EN</th><th>Angle</th><th>Vel dps</th></tr>
          </thead>
          <tbody id="liveStatusRows"></tbody>
        </table>
      </div>
      <div class="panel">
        <h2>Serial monitor</h2>
        <div id="log"></div>
        <div class="row" style="margin-top:8px">
          <input id="cmd" placeholder="Type any F401 command, for example: params" onkeydown="if(event.key==='Enter') sendManual()">
          <button onclick="sendManual()">Send</button>
          <button onclick="clearLog()">Clear log</button>
        </div>
        <div class="row">
          <label>seq id <input id="seqid" type="number" value="1"></label>
          <input id="seqcmd" value="params" style="flex:1; min-width:220px">
          <button onclick="sendSeq()">Send seq</button>
        </div>
      </div>
    </section>

    <section id="arm" class="tab">
      <div class="panel">
        <h2>ARM drive</h2>
        <div class="row">
          <button onclick="send('mode arm confirm')">Mode ARM</button>
          <button onclick="send('stop')">Stop</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
          <label><input id="liveaxis" type="checkbox"> Live axis sliders</label>
        </div>
        <div id="axisRows"></div>
      </div>
    </section>

    <section id="position" class="tab">
      <div class="panel">
        <h2>Sensors</h2>
        <div class="row">
          <button onclick="send('get sensors')">Get sensors</button>
          <button onclick="send('get as5600')">Get all AS5600</button>
          <label>CH <input id="as5600ch" type="number" min="0" max="7" value="0"></label>
          <button onclick="send('get as5600 '+v('as5600ch'))">Get CH</button>
          <label>Stream CH <input id="streamch" type="number" min="0" max="7" value="0"></label>
          <label>ms <input id="streamms" type="number" min="50" max="5000" value="250"></label>
          <button onclick="send('stream as5600 '+v('streamch')+' '+v('streamms'))">Start stream</button>
          <button onclick="send('stream off')">Stream off</button>
        </div>
      </div>
      <div id="positionRows"></div>
    </section>

    <section id="tuning" class="tab">
      <div class="panel">
        <h2>General tuning</h2>
        <div class="row">
          <label>Joint <input id="tuneAxis" type="number" min="1" max="6" value="1"></label>
          <button onclick="send('params')">Params</button>
          <button onclick="send('get motors')">Get motors</button>
          <button onclick="send('get fault')">Get fault</button>
        </div>
        <div class="grid cols2" id="tuneRows"></div>
      </div>
      <div class="panel">
        <h2>Keybinds, limits, brakepoint</h2>
        <div class="row">
          <label>KEYFWD <input id="keyfwd" type="number" value="-1"></label>
          <button onclick="setTune('keybind', v('keyfwd'))">Set keyfwd</button>
          <label>KEYBACK <input id="keyback" type="number" value="-1"></label>
          <button onclick="setTune('negkeybind', v('keyback'))">Set keyback</button>
          <label>AS5600 CH <input id="tuneAs" type="number" min="-1" max="7" value="0"></label>
          <button onclick="setTune('as5600', v('tuneAs'))">Set as5600</button>
        </div>
        <div class="row">
          <label>limits <input id="limits" type="number" min="0" max="1" value="0"></label>
          <button onclick="setTune('limits', v('limits'))">Set limits</button>
          <label>minangle <input id="minangle" value="0"></label>
          <button onclick="setTune('minangle', v('minangle'))">Set minangle</button>
          <label>maxangle <input id="maxangle" value="360"></label>
          <button onclick="setTune('maxangle', v('maxangle'))">Set maxangle</button>
          <label>brakepoint <input id="brakepoint" value="0"></label>
          <button onclick="setTune('brakepoint', v('brakepoint'))">Set brakepoint</button>
          <button onclick="setTune('brakepoint', 'off')">Brakepoint off</button>
        </div>
      </div>
    </section>

    <section id="drill" class="tab">
      <div class="panel">
        <h2>DRILL mode</h2>
        <div class="row">
          <button onclick="send('mode drill confirm')">Mode DRILL</button>
          <button onclick="send('mode safe')">Mode SAFE</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
        </div>
      </div>
      <div class="grid cols2">
        <div class="panel">
          <h2>Elevator BTS4+BTS5</h2>
          <div class="row"><label>PWM <input id="elvpwm" type="range" min="0" max="255" value="80" oninput="syncText('elvpwm','elvpwmn')"></label><input id="elvpwmn" type="number" value="80" oninput="syncText('elvpwmn','elvpwm')"></div>
          <div class="row"><button onclick="send('elv up '+v('elvpwm'))">Up</button><button onclick="send('elv down '+v('elvpwm'))">Down</button><button onclick="send('elv stop')">Stop</button></div>
          <div class="row"><label>default <input id="elvdef" type="number" value="100"></label><button onclick="send('elv default '+v('elvdef'))">Set default</button></div>
          <div class="row"><label>maxpwm <input id="elvmax" type="number" value="255"></label><button onclick="send('elv maxpwm '+v('elvmax'))">Set maxpwm</button></div>
          <div class="row"><label>invert <input id="elvinv" type="number" min="0" max="1" value="0"></label><button onclick="send('elv invert '+v('elvinv'))">Set invert</button></div>
        </div>
        <div class="panel">
          <h2>Drill BTS6</h2>
          <div class="row"><label>PWM <input id="drillpwm" type="range" min="0" max="255" value="80" oninput="syncText('drillpwm','drillpwmn')"></label><input id="drillpwmn" type="number" value="80" oninput="syncText('drillpwmn','drillpwm')"></div>
          <div class="row"><button onclick="send('drill dig '+v('drillpwm'))">Dig</button><button onclick="send('drill extract '+v('drillpwm'))">Extract</button><button onclick="send('drill stop')">Stop</button></div>
          <div class="row"><label>default <input id="drilldef" type="number" value="100"></label><button onclick="send('drill default '+v('drilldef'))">Set default</button></div>
          <div class="row"><label>maxpwm <input id="drillmax" type="number" value="255"></label><button onclick="send('drill maxpwm '+v('drillmax'))">Set maxpwm</button></div>
          <div class="row"><label>invert <input id="drillinv" type="number" min="0" max="1" value="0"></label><button onclick="send('drill invert '+v('drillinv'))">Set invert</button></div>
        </div>
      </div>
    </section>

    <section id="joy" class="tab">
      <div class="panel">
        <h2>Joystick/keybind command test</h2>
        <div class="row">
          <label>Joint <input id="joyAxis" type="number" min="1" max="6" value="1"></label>
          <label style="flex:1">Value <input id="joyValue" type="range" min="-1000" max="1000" value="0" oninput="syncText('joyValue','joyValueN')"></label>
          <input id="joyValueN" type="number" value="0" oninput="syncText('joyValueN','joyValue')">
          <button onclick="send('joy axis '+v('joyAxis')+' '+v('joyValue'))">Send joy axis</button>
          <button onclick="setBoth('joyValue','joyValueN',0); send('joy axis '+v('joyAxis')+' 0')">Zero + send</button>
        </div>
        <div class="row">
          <label>Button id <input id="joyButton" type="number" value="1000"></label>
          <button onclick="send('joy button '+v('joyButton')+' 1')">Press</button>
          <button onclick="send('joy button '+v('joyButton')+' 0')">Release</button>
        </div>
        <div class="row">
          <button onclick="send('joy button 1000 1')">Hat up press</button><button onclick="send('joy button 1000 0')">release</button>
          <button onclick="send('joy button 1001 1')">Hat down press</button><button onclick="send('joy button 1001 0')">release</button>
          <button onclick="send('joy button 1002 1')">Hat left press</button><button onclick="send('joy button 1002 0')">release</button>
          <button onclick="send('joy button 1003 1')">Hat right press</button><button onclick="send('joy button 1003 0')">release</button>
        </div>
      </div>
    </section>

    <section id="gamepad" class="tab">
      <div class="panel">
        <h2>Gamepad ARM control</h2>
        <div class="row">
          <button class="primary" onclick="startGamepadMode()">Enable gamepad TX</button>
          <button onclick="stopGamepadMode()">Pause gamepad TX</button>
          <button onclick="detectGamepads()">Detect gamepads</button>
          <button onclick="many(['mode arm confirm'])">Mode ARM</button>
          <button onclick="send('stop')">Stop ARM</button>
          <button class="danger" onclick="send('stopall')">StopAll</button>
        </div>
        <div class="row">
          <label>Gamepad <select id="gamepadSelect"></select></label>
          <label>Axis dead <input id="gpDead" type="number" min="0" max="400" value="50"></label>
          <label>Axis delta <input id="gpDelta" type="number" min="1" max="300" value="20"></label>
          <label>Poll ms <input id="gpPoll" type="number" min="10" max="200" value="20"></label>
        </div>
        <div class="small">
          Default browser mapping: left stick X -> J1 base, left stick Y -> J2 shoulder,
          right stick Y -> J3 elbow, right stick X -> J5 twist. D-pad up/down -> pitch,
          D-pad right/left -> gripper through synthetic joy button ids.
        </div>
        <div id="gamepadStatus" class="small mono" style="margin-top:8px">Gamepad TX: paused</div>
      </div>
      <div class="panel">
        <h2>Axis mapping</h2>
        <div id="gamepadAxisRows"></div>
      </div>
      <div class="panel">
        <h2>Button mapping</h2>
        <div class="small">Browser buttons 12/13/14/15 are commonly D-pad up/down/left/right.</div>
        <div id="gamepadButtonRows"></div>
      </div>
      <div class="panel">
        <h2>Live gamepad values</h2>
        <pre id="gamepadDebug" class="mono small">Press a gamepad button once, then click Detect gamepads.</pre>
      </div>
    </section>
  </main>
<script>
const AXES = ['J1 Base','J2 Shoulder','J3 Elbow','J4 Pitch','J5 Twist','J6 Gripper'];
const STATUS_FIELDS = ['PWM','DIR','EN','ANGLE','VEL_DPS'];
const MAX_BROWSER_LOG_LINES = 1200;
const tabs = [['console','Console'],['arm','ARM Drive'],['position','Position/PID'],['tuning','Tuning'],['drill','DRILL'],['joy','Joy/Keybind'],['gamepad','Gamepad ARM']];
let since = 0;
let gamepadEnabled = false;
let gamepadTimer = null;
let gamepadLastAxis = {};
let gamepadLastButtons = {};
let liveStatusRevision = -1;
let pollFailures = 0;

function q(id) { return document.getElementById(id); }
function v(id) { return q(id).value; }
function setText(id, value) {
  const node = q(id);
  if (node && node.textContent !== value) node.textContent = value;
}
function setBoth(a, b, value) { q(a).value = value; q(b).value = value; }
function syncText(source, target) { q(target).value = q(source).value; }
function tuneAxis() { return Math.max(1, Math.min(6, parseInt(v('tuneAxis') || '1'))); }
function setTune(name, value) { send(`set ${tuneAxis()} ${name} ${value}`); }

function makeLiveStatusRows() {
  const box = q('liveStatusRows');
  AXES.forEach((name, index) => {
    const row = document.createElement('tr');
    row.innerHTML = `<td>${name}</td>${STATUS_FIELDS.map(field => `<td id="liveJ${index + 1}_${field}">-</td>`).join('')}`;
    box.appendChild(row);
  });
}
function updateLiveStatus(status) {
  if (!status || !Array.isArray(status.axes)) return;
  const revision = Number.isFinite(Number(status.revision)) ? Number(status.revision) : 0;
  if (revision === liveStatusRevision) return;
  liveStatusRevision = revision;
  setText('liveMode', 'Mode: ' + (status.mode || '-'));
  setText('liveStopmode', 'Stopmode: ' + (status.stopmode || '-'));
  const receivedAt = Number(status.received_at || 0);
  const seenText = receivedAt > 0 ? new Date(receivedAt * 1000).toLocaleTimeString() : 'waiting';
  setText('liveSeen', 'Live status: ' + seenText);
  status.axes.forEach((axis, index) => {
    STATUS_FIELDS.forEach(field => {
      const cell = q(`liveJ${index + 1}_${field}`);
      const value = axis && axis[field] !== undefined ? axis[field] : '-';
      if (cell && cell.textContent !== value) cell.textContent = value;
    });
  });
}

function appendLogLines(items) {
  if (!items.length) return;
  const log = q('log');
  const fragment = document.createDocumentFragment();
  items.forEach(item => fragment.appendChild(document.createTextNode(item[1] + '\n')));
  log.appendChild(fragment);
  while (log.childNodes.length > MAX_BROWSER_LOG_LINES) log.removeChild(log.firstChild);
  log.scrollTop = log.scrollHeight;
}

function makeTabs() {
  const box = q('tabs');
  tabs.forEach(([id, label], index) => {
    const b = document.createElement('button');
    b.className = 'tab-btn' + (index === 0 ? ' active' : '');
    b.textContent = label;
    b.onclick = () => showTab(id);
    box.appendChild(b);
  });
}
function showTab(id) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.id === id));
  document.querySelectorAll('.tab-btn').forEach((b, i) => b.classList.toggle('active', tabs[i][0] === id));
}
function makeAxisRows() {
  const box = q('axisRows');
  AXES.forEach((name, idx) => {
    const axis = idx + 1;
    const row = document.createElement('div');
    row.className = 'axis-row';
    row.innerHTML = `
      <strong>${name}</strong>
      <input id="pwm${axis}" type="number" min="0" max="255" value="80">
      <span>
        <button onclick="send('forward ${axis} '+v('pwm${axis}'))">Fwd</button>
        <button onclick="send('backward ${axis} '+v('pwm${axis}'))">Back</button>
        <button onclick="send('stop ${axis}')">Stop</button>
      </span>
      <input id="axis${axis}" type="range" min="-1000" max="1000" value="0" oninput="axisSlide(${axis})" onchange="sendAxis(${axis})">
      <span>
        <button onclick="sendAxis(${axis})">Send</button>
        <button onclick="q('axis${axis}').value=0; send('axis ${axis} 0'); send('stop ${axis}')">0+Stop</button>
      </span>`;
    box.appendChild(row);
  });
}
let axisLast = Array(6).fill(0);
let axisAt = Array(6).fill(0);
function axisSlide(axis) {
  if (!q('liveaxis').checked) return;
  const value = parseInt(v('axis'+axis));
  const now = Date.now();
  if (now - axisAt[axis-1] < 60 && Math.abs(value - axisLast[axis-1]) < 40) return;
  sendAxis(axis);
}
function sendAxis(axis) {
  const value = parseInt(v('axis'+axis));
  axisLast[axis-1] = value;
  axisAt[axis-1] = Date.now();
  send(`axis ${axis} ${value}`);
}
function makePositionRows() {
  const box = q('positionRows');
  [1,2,3].forEach(axis => {
    const row = document.createElement('div');
    row.className = 'panel';
    row.innerHTML = `
      <h2>${AXES[axis-1]}</h2>
      <div class="row">
        <button onclick="send('zero ${axis}')">Zero</button>
        <label>Angle <input id="goto${axis}" value="0"></label>
        <button onclick="send('goto ${axis} '+v('goto${axis}'))">Goto</button>
        <label>Rotate <input id="rot${axis}" value="10"></label>
        <button onclick="send('rotate ${axis} '+v('rot${axis}'))">Move</button>
        <button onclick="send('stop ${axis}')">Stop</button>
        <button onclick="many(['set ${axis} stopmode hold','stop ${axis}'])">Hold now</button>
      </div>
      <div class="row">
        <label>Stopmode <select id="stopmode${axis}"><option>coast</option><option>brake</option><option selected>hold</option><option>hybrid</option></select></label>
        <button onclick="send('set ${axis} stopmode '+v('stopmode${axis}'))">Set</button>
        <label>KP <input id="kp${axis}" type="number" value="1200"></label>
        <label>KD <input id="kd${axis}" type="number" value="250"></label>
        <button onclick="many(['set ${axis} kp '+v('kp${axis}'),'set ${axis} kd '+v('kd${axis}')])">Set KP/KD</button>
      </div>
      <div class="row">
        <label>MinPWM <input id="minpwm${axis}" type="number" min="0" max="255" value="15"></label>
        <label>Tol deg <input id="tol${axis}" value="0.8"></label>
        <label>AS5600 CH <input id="as${axis}" type="number" min="-1" max="7" value="${axis-1}"></label>
        <button onclick="many(['set ${axis} minpwm '+v('minpwm${axis}'),'set ${axis} tolerance '+v('tol${axis}'),'set ${axis} as5600 '+v('as${axis}')])">Set PID/sensor</button>
        <button class="green" onclick="activeHold(${axis})">Active hold preset</button>
      </div>`;
    box.appendChild(row);
  });
}
function activeHold(axis) {
  many([
    'set heartbeat 0',
    'mode arm confirm',
    `set ${axis} as5600 ${v('as'+axis)}`,
    `set ${axis} stopmode hold`,
    `set ${axis} tolerance ${v('tol'+axis)}`,
    `set ${axis} kp ${v('kp'+axis)}`,
    `set ${axis} kd ${v('kd'+axis)}`,
    `set ${axis} minpwm ${v('minpwm'+axis)}`,
    `stop ${axis}`
  ]);
}
function makeTuneRows() {
  const box = q('tuneRows');
  const items = [
    ['invert',0,0,1], ['maxpwm',255,0,255], ['default',100,0,255],
    ['posgain',1000,100,3000], ['neggain',1000,100,3000], ['dead',50,0,400]
  ];
  items.forEach(([name, val, min, max]) => {
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `<label>${name} <input id="tune_${name}" type="range" min="${min}" max="${max}" value="${val}" oninput="syncText('tune_${name}','tune_${name}_n')"></label><input id="tune_${name}_n" type="number" min="${min}" max="${max}" value="${val}" oninput="syncText('tune_${name}_n','tune_${name}')"><button onclick="setTune('${name}', v('tune_${name}'))">Set</button>`;
    box.appendChild(row);
  });
}
function makeGamepadRows() {
  const axisDefaults = [
    ['Base', 1, 0, false],
    ['Shoulder', 2, 1, true],
    ['Elbow', 3, 3, true],
    ['Twist', 5, 2, false],
  ];
  const axisBox = q('gamepadAxisRows');
  axisDefaults.forEach((item, index) => {
    const [name, joint, gpAxis, invert] = item;
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `
      <strong style="width:86px">${name}</strong>
      <label>Joint <input id="gpAxisJoint${index}" type="number" min="1" max="6" value="${joint}"></label>
      <label>Gamepad axis <input id="gpAxisId${index}" type="number" min="0" max="15" value="${gpAxis}"></label>
      <label><input id="gpAxisInv${index}" type="checkbox" ${invert ? 'checked' : ''}> Invert</label>
      <button onclick="send('axis '+v('gpAxisJoint${index}')+' 0')">Zero this joint</button>`;
    axisBox.appendChild(row);
  });
  const buttonDefaults = [
    ['Pitch up', 12, 1000],
    ['Pitch down', 13, 1001],
    ['Gripper left', 14, 1002],
    ['Gripper right', 15, 1003],
  ];
  const buttonBox = q('gamepadButtonRows');
  buttonDefaults.forEach((item, index) => {
    const [name, button, joyId] = item;
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `
      <strong style="width:100px">${name}</strong>
      <label>Browser button <input id="gpBtnId${index}" type="number" min="0" max="63" value="${button}"></label>
      <label>F401 joy id <input id="gpJoyId${index}" type="number" min="0" max="32767" value="${joyId}"></label>
      <button onclick="send('joy button '+v('gpJoyId${index}')+' 1')">Press test</button>
      <button onclick="send('joy button '+v('gpJoyId${index}')+' 0')">Release test</button>`;
    buttonBox.appendChild(row);
  });
}
async function api(path, data) {
  const options = data === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data)};
  const res = await fetch(path, options);
  if (!res.ok) throw new Error(await res.text());
  return await res.json();
}
async function send(cmd) {
  if (!cmd || !cmd.trim()) return;
  await api('/api/send', {cmd});
}
async function many(cmds) { await api('/api/send_many', {cmds}); }
function sendManual() {
  const cmd = v('cmd');
  q('cmd').value = '';
  send(cmd);
}
function sendSeq() {
  const id = parseInt(v('seqid') || '1');
  send(`seq ${id} ${v('seqcmd')}`);
  q('seqid').value = id + 1;
}
function factoryReset() {
  if (confirm('Reset RAM settings to firmware defaults?')) send('factoryreset confirm');
}
async function refreshPorts() {
  const data = await api('/api/ports');
  const box = q('port');
  const old = box.value;
  box.innerHTML = '';
  data.ports.forEach(p => {
    const opt = document.createElement('option');
    opt.value = p; opt.textContent = p; box.appendChild(opt);
  });
  if (old) box.value = old;
}
async function connect() {
  await api('/api/connect', {port:v('port'), baud:parseInt(v('baud')), dry_run:q('dry').checked});
}
async function disconnect() { await api('/api/disconnect', {}); }
async function setHeartbeat() { await api('/api/auto_heartbeat', {enabled:q('autohb').checked}); }
function detectGamepads() {
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  const box = q('gamepadSelect');
  const old = box.value;
  box.innerHTML = '';
  pads.forEach(pad => {
    const opt = document.createElement('option');
    opt.value = pad.index;
    opt.textContent = `${pad.index}: ${pad.id}`;
    box.appendChild(opt);
  });
  if (old) box.value = old;
  q('gamepadStatus').textContent = pads.length ? `Gamepads detected: ${pads.length}` : 'No gamepad detected. Press a gamepad button, then Detect.';
}
function selectedGamepad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const selected = parseInt(v('gamepadSelect') || '0');
  return pads[selected] || Array.from(pads).find(Boolean) || null;
}
function scaledGamepadAxis(raw, invert) {
  let value = invert ? -raw : raw;
  if (value > 1) value = 1;
  if (value < -1) value = -1;
  let scaled = Math.round(value * 1000);
  const dead = Math.max(0, Math.min(400, parseInt(v('gpDead') || '50')));
  if (Math.abs(scaled) <= dead) scaled = 0;
  return scaled;
}
function gamepadAxisMappings() {
  const rows = [];
  for (let i = 0; i < 4; i++) {
    rows.push({
      joint: parseInt(v('gpAxisJoint'+i) || '1'),
      axis: parseInt(v('gpAxisId'+i) || '0'),
      invert: q('gpAxisInv'+i).checked
    });
  }
  return rows;
}
function gamepadButtonMappings() {
  const rows = [];
  for (let i = 0; i < 4; i++) {
    rows.push({
      button: parseInt(v('gpBtnId'+i) || '0'),
      joy: parseInt(v('gpJoyId'+i) || '0')
    });
  }
  return rows;
}
function startGamepadMode() {
  detectGamepads();
  gamepadEnabled = true;
  gamepadLastAxis = {};
  gamepadLastButtons = {};
  q('gamepadStatus').textContent = 'Gamepad TX: running';
  many(['mode arm confirm']);
  if (gamepadTimer === null) gamepadLoop();
}
function stopGamepadMode() {
  gamepadEnabled = false;
  q('gamepadStatus').textContent = 'Gamepad TX: paused, stop sent';
  send('stop');
}
function gamepadLoop() {
  if (gamepadEnabled) {
    const pad = selectedGamepad();
    if (!pad) {
      q('gamepadStatus').textContent = 'Gamepad TX: running, no gamepad visible';
    } else {
      const delta = Math.max(1, parseInt(v('gpDelta') || '20'));
      gamepadAxisMappings().forEach(map => {
        if (!Number.isFinite(map.joint) || map.joint < 1 || map.joint > 6) return;
        if (!Number.isFinite(map.axis) || map.axis < 0 || map.axis >= pad.axes.length) return;
        const value = scaledGamepadAxis(pad.axes[map.axis], map.invert);
        const key = `j${map.joint}`;
        const prev = gamepadLastAxis[key];
        if (prev === undefined || Math.abs(value - prev) >= delta || (prev !== 0 && value === 0)) {
          gamepadLastAxis[key] = value;
          send(`axis ${map.joint} ${value}`);
        }
      });
      gamepadButtonMappings().forEach(map => {
        if (!Number.isFinite(map.button) || map.button < 0 || map.button >= pad.buttons.length) return;
        const pressed = pad.buttons[map.button].pressed ? 1 : 0;
        const key = `b${map.button}->${map.joy}`;
        const prev = gamepadLastButtons[key] || 0;
        if (pressed !== prev) {
          gamepadLastButtons[key] = pressed;
          send(`joy button ${map.joy} ${pressed}`);
        }
      });
      const axes = pad.axes.map((value, index) => `${index}:${value.toFixed(2)}`).join('  ');
      const buttons = pad.buttons.map((button, index) => button.pressed ? index : null).filter(v => v !== null).join(', ');
      q('gamepadStatus').textContent = `Gamepad TX: running - ${pad.id}`;
      q('gamepadDebug').textContent = `axes ${axes}\npressed buttons ${buttons || 'none'}`;
    }
  }
  const pollMs = Math.max(10, Math.min(200, parseInt(v('gpPoll') || '20')));
  gamepadTimer = setTimeout(gamepadLoop, pollMs);
}
async function poll() {
  try {
    const data = await api('/api/logs?since='+since);
    pollFailures = 0;
    since = data.next;
    setText('connected', data.connected ? 'Connected' : 'Disconnected');
    setText('mode', 'Mode: ' + data.mode);
    setText('fault', 'Fault: ' + data.fault);
    updateLiveStatus(data.live_status);
    appendLogLines(data.lines);
  } catch (err) {
    pollFailures += 1;
    if (pollFailures >= 3) setText('connected', 'Server error');
  }
  setTimeout(poll, 250);
}
function clearLog() { q('log').replaceChildren(); }
async function init() {
  makeTabs();
  makeLiveStatusRows();
  makeAxisRows();
  makePositionRows();
  makeTuneRows();
  makeGamepadRows();
  await refreshPorts();
  const initData = await api('/api/init');
  q('baud').value = initData.baud;
  q('dry').checked = initData.dry_run;
  q('autohb').checked = initData.auto_heartbeat;
  if (initData.port) q('port').value = initData.port;
  updateLiveStatus(initData.live_status);
  poll();
}
init();
</script>
</body>
</html>
"""


class WebGuiState:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.link = SerialLink(self.log)
        self.logs: List[tuple[int, str]] = []
        self.next_log_id = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.auto_heartbeat = True
        self.mode = "unknown"
        self.fault = "unknown"
        self.live_status = blank_status_snapshot()
        self.live_status_revision = 0
        self.live_status_received_at = 0.0
        self.port = args.port or ""
        self.baud = int(args.baud)
        self.dry_run = bool(args.dry_run)

    def log(self, text: str) -> None:
        with self.lock:
            self.logs.append((self.next_log_id, text))
            self.next_log_id += 1
            if len(self.logs) > 2000:
                self.logs = self.logs[-1200:]

    def connect(self, port: str, baud: int, dry_run: bool) -> None:
        self.link.connect(port, baud, dry_run)
        with self.lock:
            self.port = port
            self.baud = baud
            self.dry_run = dry_run

    def disconnect(self) -> None:
        self.link.close()
        self.log("LOCAL: disconnected")

    def send(self, command: str) -> None:
        self.link.send(command)

    def send_many(self, commands: Iterable[str]) -> None:
        for command in commands:
            self.send(command)

    def lines_since(self, since: int) -> List[tuple[int, str]]:
        with self.lock:
            return [item for item in self.logs if item[0] >= since]

    def next_id(self) -> int:
        with self.lock:
            return self.next_log_id

    def live_status_snapshot(self) -> Dict[str, object]:
        with self.lock:
            return self._live_status_snapshot_locked()

    def _live_status_snapshot_locked(self) -> Dict[str, object]:
        axes = self.live_status.get("axes", [])
        if isinstance(axes, list):
            axes_copy = [axis.copy() if isinstance(axis, dict) else {} for axis in axes]
        else:
            axes_copy = blank_status_axes()
        return {
            "mode": self.live_status.get("mode", "-"),
            "stopmode": self.live_status.get("stopmode", "-"),
            "axes": axes_copy,
            "revision": self.live_status_revision,
            "received_at": self.live_status_received_at,
        }

    def init_snapshot(self) -> Dict[str, object]:
        with self.lock:
            return {
                "port": self.port,
                "baud": self.baud,
                "dry_run": self.dry_run,
                "auto_heartbeat": self.auto_heartbeat,
                "live_status": self._live_status_snapshot_locked(),
            }

    def poll_snapshot(self, since: int) -> Dict[str, object]:
        connected = self.link.is_open()
        with self.lock:
            return {
                "lines": [item for item in self.logs if item[0] >= since],
                "next": self.next_log_id,
                "connected": connected,
                "mode": self.mode,
                "fault": self.fault,
                "live_status": self._live_status_snapshot_locked(),
            }

    def set_auto_heartbeat(self, enabled: bool) -> None:
        with self.lock:
            self.auto_heartbeat = enabled

    def update_status_from_line(self, line: str) -> bool:
        if line == "OK HEARTBEAT":
            return True
        status = parse_status_line(line)
        if status is not None:
            with self.lock:
                self.live_status = status
                self.live_status_revision += 1
                self.live_status_received_at = time.time()
                self.mode = str(status.get("mode", "-"))
            return True
        if line.startswith("OK MODE "):
            with self.lock:
                self.mode = line.removeprefix("OK MODE ").strip()
        elif line.startswith("OK FAULT "):
            with self.lock:
                self.fault = line.removeprefix("OK FAULT ").strip()
        elif "HEARTBEAT_TIMEOUT" in line:
            with self.lock:
                self.fault = "heartbeat timeout"
        return False

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self._poll_loop, daemon=True)
        thread.start()
        return thread

    def _poll_loop(self) -> None:
        last_heartbeat = time.monotonic()
        while not self.stop_event.is_set():
            for line in self.link.read_lines():
                if not self.update_status_from_line(line):
                    self.log(line if line.startswith("LOCAL:") else f"F401: {line}")
            now = time.monotonic()
            with self.lock:
                auto_heartbeat = self.auto_heartbeat
            if auto_heartbeat and self.link.is_open() and now - last_heartbeat >= 0.5:
                self.link.send("heartbeat", log_tx=False)
                last_heartbeat = now
            time.sleep(0.03)

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.link.is_open():
            try:
                self.link.send("stopall")
            finally:
                self.link.close()


def make_web_handler(state: WebGuiState) -> type[BaseHTTPRequestHandler]:
    class WebHandler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: object) -> None:
            return

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send_bytes(WEB_HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif parsed.path == "/api/init":
                self._send_json(state.init_snapshot())
            elif parsed.path == "/api/ports":
                self._send_json({"ports": serial_ports()})
            elif parsed.path == "/api/logs":
                since_text = parse_qs(parsed.query).get("since", ["0"])[0]
                try:
                    since = int(since_text)
                except ValueError:
                    since = 0
                self._send_json(state.poll_snapshot(since))
            else:
                self.send_error(404, "not found")

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            try:
                data = self._read_json()
                if parsed.path == "/api/send":
                    state.send(str(data.get("cmd", "")))
                    self._send_json({"ok": True})
                elif parsed.path == "/api/send_many":
                    commands = data.get("cmds", [])
                    if not isinstance(commands, list):
                        raise ValueError("cmds must be a list")
                    state.send_many(str(command) for command in commands)
                    self._send_json({"ok": True})
                elif parsed.path == "/api/connect":
                    state.connect(
                        str(data.get("port", "")),
                        int(data.get("baud", DEFAULT_BAUD)),
                        bool(data.get("dry_run", False)),
                    )
                    self._send_json({"ok": True})
                elif parsed.path == "/api/disconnect":
                    state.disconnect()
                    self._send_json({"ok": True})
                elif parsed.path == "/api/auto_heartbeat":
                    state.set_auto_heartbeat(bool(data.get("enabled", True)))
                    self._send_json({"ok": True})
                else:
                    self.send_error(404, "not found")
            except Exception as exc:
                self.send_response(400)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(str(exc).encode("utf-8", errors="replace"))

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length else b"{}"
            if not raw:
                return {}
            decoded = raw.decode("utf-8", errors="replace")
            data = json.loads(decoded)
            if not isinstance(data, dict):
                raise ValueError("request body must be a JSON object")
            return data

        def _send_json(self, data: object) -> None:
            self._send_bytes(json.dumps(data).encode("utf-8"), "application/json; charset=utf-8")

        def _send_bytes(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    return WebHandler


def run_web_gui(args: argparse.Namespace) -> int:
    state = WebGuiState(args)
    if args.auto_connect or args.dry_run:
        try:
            state.connect(args.port or "", int(args.baud), bool(args.dry_run))
        except Exception as exc:
            state.log(f"LOCAL: auto-connect failed: {exc}")
    state.start()

    server = ThreadingHTTPServer((args.web_host, args.web_port), make_web_handler(state))
    url = f"http://{args.web_host}:{args.web_port}/"
    print(f"Web GUI ready: {url}")
    print("Close with Ctrl+C. Shutdown sends stopall.")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping web GUI...")
    finally:
        state.shutdown()
        server.server_close()
    return 0


def run_self_test() -> None:
    class FakeSerial:
        def __init__(self) -> None:
            self.rx = bytearray()
            self.tx = bytearray()
            self.closed = False

        @property
        def in_waiting(self) -> int:
            return len(self.rx)

        def feed(self, data: bytes) -> None:
            self.rx.extend(data)

        def read(self, size: int) -> bytes:
            data = bytes(self.rx[:size])
            del self.rx[:size]
            return data

        def write(self, data: bytes) -> int:
            self.tx.extend(data)
            return len(data)

        def close(self) -> None:
            self.closed = True

    assert clamp_int(300, 0, 255) == 255
    assert clamp_int(-5, 0, 255) == 0
    assert clamp_int(120, 0, 255) == 120
    assert STOP_MODES == ("coast", "brake", "hold", "hybrid")
    assert AXIS_NAMES[3] == "J4 Pitch"
    status_axes = [
        "J1_PWM:120 J1_DIR:forward J1_EN:1 J1_ANGLE:12.34 J1_VEL_DPS:-5.67"
    ]
    status_axes.extend(
        f"J{axis}_PWM:0 J{axis}_DIR:stop J{axis}_EN:0 J{axis}_ANGLE:NA J{axis}_VEL_DPS:NA"
        for axis in range(2, 7)
    )
    parsed = parse_status_line("MODE:arm STOPMODE:coast " + " ".join(status_axes))
    assert parsed is not None
    assert parsed["mode"] == "arm"
    assert parsed["stopmode"] == "coast"
    axes = parsed["axes"]
    assert isinstance(axes, list)
    assert axes[0]["PWM"] == "120"
    assert axes[0]["VEL_DPS"] == "-5.67"
    assert axes[5]["ANGLE"] == "NA"
    assert parse_status_line("MODE:arm STOPMODE:coast J1_PWM:120") is None
    corrupt_status = "MODE:arm STOPMODE:coast " + " ".join(status_axes).replace("J3_PWM:0", "J3_PWM:999")
    assert parse_status_line(corrupt_status) is None

    state_args = argparse.Namespace(port=None, baud=DEFAULT_BAUD, dry_run=True)
    web_state = WebGuiState(state_args)
    web_state.connect("", DEFAULT_BAUD, True)
    assert web_state.update_status_from_line("MODE:arm STOPMODE:coast " + " ".join(status_axes))
    live_snapshot = web_state.live_status_snapshot()
    assert live_snapshot["revision"] == 1
    assert live_snapshot["received_at"] > 0
    poll_snapshot = web_state.poll_snapshot(0)
    assert poll_snapshot["connected"]
    assert poll_snapshot["next"] == len(poll_snapshot["lines"])
    web_state.link.close()
    assert "stopall" in WEB_HTML
    assert "Active hold preset" in WEB_HTML
    assert "liveStatusRows" in WEB_HTML

    fake = FakeSerial()
    serial_log: List[str] = []
    link = SerialLink(serial_log.append)
    link.serial = fake
    fake.feed(b"MODE:arm STOPMODE:co")
    assert link.read_lines() == []
    fake.feed(b"ast J1_PWM:12\r\nOK FIRST\nOK SECOND\r\n")
    assert link.read_lines(max_lines=2) == ["MODE:arm STOPMODE:coast J1_PWM:12", "OK FIRST"]
    assert link.read_lines() == ["OK SECOND"]
    link.send("heartbeat", log_tx=False)
    assert fake.tx == b"heartbeat\n"

    fake.feed(b"x" * (MAX_SERIAL_LINE_BYTES + 1))
    assert link.read_lines() == []
    assert link.read_lines() == [
        f"LOCAL: discarded serial line longer than {MAX_SERIAL_LINE_BYTES} bytes"
    ]
    fake.feed(b"discarded suffix\nOK RECOVERED\n")
    assert link.read_lines() == ["OK RECOVERED"]
    link.close()
    assert fake.closed
    print("self-test ok")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test GUI for the F401 manipulation/drill firmware.")
    parser.add_argument("--port", help="FTDI serial port, for example COM7.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--dry-run", action="store_true", help="Open GUI without serial hardware; only log TX lines.")
    parser.add_argument("--auto-connect", action="store_true", help="Connect automatically on startup.")
    parser.add_argument("--web", action="store_true", help="Force browser-based GUI instead of Tkinter.")
    parser.add_argument("--web-host", default=DEFAULT_WEB_HOST)
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT)
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically open the browser for --web.")
    parser.add_argument("--list-ports", action="store_true", help="Print serial ports and exit.")
    parser.add_argument("--self-test", action="store_true", help="Run non-GUI checks and exit.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if args.list_ports:
        for port in serial_ports():
            print(port)
        return 0

    if args.web or tk is None:
        if tk is None and not args.web:
            print("tkinter is not available in this Python. Starting browser-based GUI instead.")
        return run_web_gui(args)

    root = tk.Tk()
    F401TestGui(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
