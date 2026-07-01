#!/usr/bin/env python3
"""
Earendil — Lightweight Rover Control GUI for STM32H723ZG
========================================================
Single-file PySide6 + pyserial application optimized for Raspberry Pi 5.
Communicates with the H7 firmware over USART3 / ST-LINK VCP.

Controls:
    W/A/S/D   -> forward / left / backward / right
    Space     -> stop
    X         -> brake
    M         -> toggle mode (RPM / DUTY)
    LShift    -> increase RPM/DUTY by +5
    LCtrl     -> decrease RPM/DUTY by -5
"""

import re
import sys
import time
from collections import deque

# ── Dependency check ───────────────────────────────────────────────────────
_missing = []
try:
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QPushButton, QComboBox, QLineEdit,
        QGroupBox, QTextEdit, QTableWidget, QTableWidgetItem,
        QDialog, QFormLayout, QGridLayout, QHeaderView,
    )
    from PySide6.QtCore import Qt, QTimer, Signal, QObject, QThread, QEvent
    from PySide6.QtGui import (
        QTextCursor, QKeyEvent,
    )
except ImportError:
    _missing.append("PySide6  (pip install PySide6)")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    _missing.append("pyserial  (pip install pyserial)")

if _missing:
    print("ERROR: Missing required packages:")
    for m in _missing:
        print(f"  - {m}")
    sys.exit(1)


# ----------------------------------------------------------------------------
#  Configuration Constants
# ----------------------------------------------------------------------------

MAX_CONSOLE_LINES = 300
TELEMETRY_UI_UPDATE_MS = 200

# ----------------------------------------------------------------------------
#  Serial Reader Thread
# ----------------------------------------------------------------------------

class SerialReaderThread(QThread):
    """Background thread that reads lines from the serial port."""

    line_received = Signal(str)
    error_occurred = Signal(str)
    disconnected = Signal()

    def __init__(self, ser: serial.Serial):
        super().__init__()
        self.ser = ser
        self._running = True

    def run(self):
        buf = b""
        while self._running:
            try:
                if self.ser and self.ser.is_open:
                    data = self.ser.read(self.ser.in_waiting or 1)
                    if data:
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            text = line.decode("utf-8", errors="replace").strip()
                            if text:
                                self.line_received.emit(text)
                    else:
                        self.msleep(5)
                else:
                    break
            except serial.SerialException:
                if self._running:
                    self.disconnected.emit()
                break
            except Exception as e:
                if self._running:
                    self.error_occurred.emit(str(e))
                break

    def stop(self):
        self._running = False
        self.wait(2000)


# ----------------------------------------------------------------------------
#  H7 UART Error Log Parsing
# ----------------------------------------------------------------------------

_RE_UART_ERROR_CODE = re.compile(
    r"^\[ERROR\]\s+(USART2|UART4|UART5|UART7)\s+"
    r"UART error (?:code|still unresolved):\s+(0x[0-9A-Fa-f]+)$"
)
_RE_UART_ERROR_DECODED = re.compile(
    r"^\[ERROR\]\s+(USART2|UART4|UART5|UART7)\s+error:\s+(.+)$"
)
_RE_UART_RECOVERED = re.compile(
    r"^\[INFO\]\s+(USART2|UART4|UART5|UART7)\s+RX recovered after UART error$"
)

# ── F411 motor telemetry patterns ──────────────────────────────────────────
_RE_MOTOR_TEL_TAGGED = re.compile(
    r"(?:\[INFO\]\s*)?\[TEL\]\[(FL|FR|RL|RR)\]\s+(RPM:.*)$"
)
_RE_MOTOR_TEL_UART = re.compile(
    r"(?:\[INFO\]\s*)?\[(USART2_RX|UART4_RX|UART5_RX|UART7_RX)\]\s+(RPM:.*)$"
)

# ── Operating mode confirmation from H7 firmware ──────────────────────────
_RE_OP_MODE_CONFIRM = re.compile(
    r"\[MODE\]\s+(DISARM|MANUAL|AUTONOMOUS)\s+active\b"
)


# ----------------------------------------------------------------------------
#  F411 Motor Tuning Settings Dialog
# ----------------------------------------------------------------------------

class MotorSettingsDialog(QDialog):
    """F411 Motor Tuning Settings dialog — lightweight version."""

    NUM_SLOTS = 8
    MOTOR_TAGS = ("FL", "FR", "RL", "RR")

    def __init__(self, main_gui: "EarendilControlGui", parent=None):
        super().__init__(parent)
        self._gui = main_gui
        self.setWindowTitle("F411 Motor Tuning Settings")
        self.setMinimumWidth(560)

        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(12, 12, 12, 12)

        root.addWidget(self._build_tuning_table_group())
        root.addWidget(self._build_form_group())
        root.addLayout(self._build_send_row())
        root.addLayout(self._build_read_row())
        root.addLayout(self._build_utility_row())

        self._defaults = {
            "base":      ["300", "500", "800", "1200", "1700", "2200", "2800", "3500"],
            "boost":     ["1300", "1600", "1700", "2000", "2600", "3200", "3800", "4000"],
            "boostms":   "150",
            "kick_duty": "960",
            "kick_ms":   "50",
            "ramp_up":   "150",
            "ramp_down": "150",
            "kp":        "10",
            "ki":        "10",
            "telper":    "1",
        }
        self._reset_fields(log=False)

    def _build_tuning_table_group(self) -> QGroupBox:
        grp = QGroupBox("Tuning Slots")
        lay = QGridLayout(grp)
        lay.setSpacing(6)
        lay.setContentsMargins(10, 14, 10, 10)

        lay.addWidget(QLabel("Slot:"), 0, 0)
        for i in range(self.NUM_SLOTS):
            lbl = QLabel(str(i + 1))
            lbl.setAlignment(Qt.AlignCenter)
            lay.addWidget(lbl, 0, i + 1)

        lay.addWidget(QLabel("Base PWM:"), 1, 0)
        self._base_edits = []
        for i in range(self.NUM_SLOTS):
            edit = QLineEdit()
            edit.setPlaceholderText("0")
            edit.setFixedWidth(60)
            edit.setAlignment(Qt.AlignCenter)
            lay.addWidget(edit, 1, i + 1)
            self._base_edits.append(edit)

        lay.addWidget(QLabel("Boost PWM:"), 2, 0)
        self._boost_edits = []
        for i in range(self.NUM_SLOTS):
            edit = QLineEdit()
            edit.setPlaceholderText("0")
            edit.setFixedWidth(60)
            edit.setAlignment(Qt.AlignCenter)
            lay.addWidget(edit, 2, i + 1)
            self._boost_edits.append(edit)

        return grp

    def _build_form_group(self) -> QGroupBox:
        grp = QGroupBox("Boost / Kick / Ramp / PI / Telemetry")
        form = QFormLayout(grp)
        form.setSpacing(6)
        form.setContentsMargins(10, 14, 10, 10)

        self._boostms_edit  = QLineEdit(); self._boostms_edit.setPlaceholderText("0")
        self._kick_duty_edit = QLineEdit(); self._kick_duty_edit.setPlaceholderText("0")
        self._kick_ms_edit  = QLineEdit(); self._kick_ms_edit.setPlaceholderText("0")
        self._ramp_up_edit  = QLineEdit(); self._ramp_up_edit.setPlaceholderText("0")
        self._ramp_dn_edit  = QLineEdit(); self._ramp_dn_edit.setPlaceholderText("0")
        self._kp_edit       = QLineEdit(); self._kp_edit.setPlaceholderText("0")
        self._ki_edit       = QLineEdit(); self._ki_edit.setPlaceholderText("0")
        self._telper_edit   = QLineEdit(); self._telper_edit.setPlaceholderText("100")

        form.addRow("Boost MS:",         self._boostms_edit)
        form.addRow("Kick Duty:",         self._kick_duty_edit)
        form.addRow("Kick MS:",           self._kick_ms_edit)
        form.addRow("Ramp Up:",          self._ramp_up_edit)
        form.addRow("Ramp Down:",         self._ramp_dn_edit)
        form.addRow("Kp:",                self._kp_edit)
        form.addRow("Ki:",                self._ki_edit)
        form.addRow("Telemetry Period:", self._telper_edit)
        return grp

    def _build_send_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(6)
        self._send_buttons: dict[str, QPushButton] = {}
        for motor in self.MOTOR_TAGS + ("All",):
            btn = QPushButton(f"Send to {motor}")
            btn.clicked.connect(lambda _=False, m=motor: self._on_send_to(motor))
            row.addWidget(btn, 1)
            self._send_buttons[motor] = btn
        return row

    def _build_utility_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(6)
        self._btn_reset = QPushButton("Reset Fields")
        self._btn_reset.clicked.connect(self._reset_fields)
        row.addWidget(self._btn_reset, 1)
        self._btn_close = QPushButton("Close")
        self._btn_close.clicked.connect(self.accept)
        row.addWidget(self._btn_close, 1)
        return row

    def _build_read_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(6)
        for motor in self.MOTOR_TAGS:
            btn = QPushButton(f"Read {motor}")
            row.addWidget(btn, 1)
        return row

    def collect_f411_tuning_settings(self) -> dict:
        return {
            "base":      [e.text().strip() for e in self._base_edits],
            "boost":     [e.text().strip() for e in self._boost_edits],
            "boostms":   self._boostms_edit.text().strip(),
            "kick_duty": self._kick_duty_edit.text().strip(),
            "kick_ms":   self._kick_ms_edit.text().strip(),
            "ramp_up":   self._ramp_up_edit.text().strip(),
            "ramp_down": self._ramp_dn_edit.text().strip(),
            "kp":        self._kp_edit.text().strip(),
            "ki":        self._ki_edit.text().strip(),
            "telper":    self._telper_edit.text().strip(),
        }

    def _resolve_motors(self, key: str) -> list:
        if key == "All":
            return ["ALL"]
        return [key] if key in self.MOTOR_TAGS else []

    def _reset_fields(self, log: bool = True):
        d = self._defaults
        for i in range(self.NUM_SLOTS):
            self._base_edits[i].setText(d["base"][i])
            self._boost_edits[i].setText(d["boost"][i])
        self._boostms_edit.setText(d["boostms"])
        self._kick_duty_edit.setText(d["kick_duty"])
        self._kick_ms_edit.setText(d["kick_ms"])
        self._ramp_up_edit.setText(d["ramp_up"])
        self._ramp_dn_edit.setText(d["ramp_down"])
        self._kp_edit.setText(d["kp"])
        self._ki_edit.setText(d["ki"])
        self._telper_edit.setText(d["telper"])
        if log:
            self._gui._log_info("[F411-TUNE] Settings reset to defaults")

    def _on_send_to(self, target: str):
        motors = self._resolve_motors(target)
        if not motors:
            return
        settings = self.collect_f411_tuning_settings()
        all_cmds: list[str] = []
        for motor in motors:
            if motor == "ALL":
                all_cmds.append("stop")
            else:
                all_cmds.append(f"{motor} stop")
            cmds = self._gui.build_f411_tuning_commands(motor, settings)
            all_cmds.extend(cmds)
            if motor == "ALL":
                all_cmds.extend(["FL spstat", "FR spstat", "RL spstat", "RR spstat"])
            else:
                all_cmds.append(f"{motor} spstat")
        self._gui.enqueue_f411_tuning_sequence(all_cmds, dialog=self)

    def _set_send_buttons_enabled(self, enabled: bool):
        for btn in self._send_buttons.values():
            btn.setEnabled(enabled)
        if hasattr(self, "_btn_reset"):
            self._btn_reset.setEnabled(enabled)


# ----------------------------------------------------------------------------
#  Main GUI
# ----------------------------------------------------------------------------

class EarendilControlGui(QMainWindow):
    """Lightweight rover control GUI optimized for Raspberry Pi 5."""

    REPEAT_INTERVAL_MS = 500
    TUNING_SEND_INTERVAL_MS = 100
    OP_MODE_CONFIRM_TIMEOUT_MS = 3000
    DEFAULT_RPM = 100
    DEFAULT_PWM = 100
    RPM_MAX = 200
    PWM_MAX = 4000
    VALUE_STEP = 5

    MOTOR_ROW = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}

    UART_TO_MOTOR = {
        "USART2": "FL",
        "UART4":  "FR",
        "UART7":  "RL",
        "UART5":  "RR",
    }

    UART_ERROR_CODES = {"FE", "NE", "ORE", "PE", "DMA", "RTO"}

    MOTOR_COL = {
        "motor":        0,
        "current_rpm":  1,
        "target_rpm":   2,
        "pwm":          3,
        "direction":    4,
        "control_mode": 5,
        "brake_status": 6,
        "fault_code":   7,
        "error":        8,
        "link":         9,
    }
    MOTOR_COL_HEADERS = [
        "Motor", "RPM", "Target", "PWM", "Dir",
        "Mode", "Brake", "Fault", "Err", "Link",
    ]

    UART_RX_TO_MOTOR = {
        "USART2_RX": "FL",
        "UART4_RX":  "FR",
        "UART7_RX":  "RL",
        "UART5_RX":  "RR",
    }

    _APP_PH_MAP = {"0": "Stopped", "1": "Running", "2": "Brake", "3": "Idle", "4": "Error"}
    _DIR_MAP = {"F": "Fwd", "R": "Rev", "N": "Neutral"}
    _SP_MAP = {"0": "Duty", "1": "RPM"}
    _BRAKE_MAP = {"0": "Off", "1": "Active"}

    OPERATING_MODES = {
        "disarm":  {"label": "DISARM",     "command": "mode disarm",  "status_bg": "#B00020", "status_fg": "#FFFFFF"},
        "manual":  {"label": "MANUAL",     "command": "mode manual",  "status_bg": "#FFD66B", "status_fg": "#101014"},
        "auto":    {"label": "AUTONOMOUS", "command": "mode auto",    "status_bg": "#1E8E3E", "status_fg": "#FFFFFF"},
    }

    MOVE_KEYS = {
        Qt.Key_W: "W",
        Qt.Key_S: "S",
        Qt.Key_A: "A",
        Qt.Key_D: "D",
    }
    MOVEMENT_KEYS = ("W", "S", "A", "D")

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Earendil — Rover Control")
        self.setMinimumSize(900, 580)

        self.ser: serial.Serial | None = None
        self.reader_thread: SerialReaderThread | None = None
        self.connected = False
        self.mode = "RPM"
        self.current_rpm = self.DEFAULT_RPM
        self.current_pwm = self.DEFAULT_PWM

        self._operating_mode = "disarm"
        self._pending_mode: str | None = None
        self._active_move_key: str | None = None
        self._move_held: set[str] = set()
        self._move_order: deque[str] = deque()
        self._active_modifier: str | None = None
        self._keys_held: set[str] = set()

        self._motor_uart_error_text: dict[str, str] = {"FL": "", "FR": "", "RL": "", "RR": ""}
        self._uart_report_decoded: dict[str, list[str]] = {}
        self._motor_fault_code: dict[str, str] = {"FL": "0", "FR": "0", "RL": "0", "RR": "0"}
        self._motor_telemetry: dict[str, dict[str, str]] = {
            m: {} for m in ("FL", "FR", "RL", "RR")
        }

        self._tuning_send_queue: deque[str] = deque()
        self._tuning_dialog_ref = None
        self._tuning_send_timer = QTimer(self)
        self._tuning_send_timer.setInterval(self.TUNING_SEND_INTERVAL_MS)
        self._tuning_send_timer.timeout.connect(self._send_next_f411_tuning_command)

        self._telemetry_buffer: dict[str, dict[str, str]] = {
            m: {} for m in ("FL", "FR", "RL", "RR")
        }
        self._telemetry_dirty = False
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.setInterval(TELEMETRY_UI_UPDATE_MS)
        self._telemetry_timer.timeout.connect(self._flush_telemetry_table)

        self._build_ui()
        self._apply_stylesheet()

        self._repeat_timer = QTimer(self)
        self._repeat_timer.setInterval(self.REPEAT_INTERVAL_MS)
        self._repeat_timer.timeout.connect(self._repeat_movement)

        self._pending_mode_timer = QTimer(self)
        self._pending_mode_timer.setSingleShot(True)
        self._pending_mode_timer.setInterval(self.OP_MODE_CONFIRM_TIMEOUT_MS)
        self._pending_mode_timer.timeout.connect(self._on_pending_mode_timeout)

        for btn in self.findChildren(QPushButton):
            btn.setFocusPolicy(Qt.NoFocus)
        self._h7_input.installEventFilter(self)
        self.setFocusPolicy(Qt.StrongFocus)

        self._log_info("Ready. Connect to rover to begin.")

    # ----------------------------------------------------------------------
    #  UI Construction
    # ----------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        row1 = QHBoxLayout()
        row1.addWidget(self._build_connection_group())
        root.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(self._build_mode_value_group(), 1)
        row2.addWidget(self._build_operating_mode_group())
        root.addLayout(row2)

        root.addWidget(self._build_motor_table_group())

        root.addWidget(self._build_console_group())

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("Serial Connection")
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(4)

        lay.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setFixedWidth(175)
        lay.addWidget(self._port_combo)

        self._btn_refresh = QPushButton("Refresh")
        self._btn_refresh.setFixedWidth(80)
        self._btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(self._btn_refresh)

        lay.addWidget(QLabel("Baud:"))
        self._baud_edit = QLineEdit("115200")
        self._baud_edit.setFixedWidth(90)
        lay.addWidget(self._baud_edit)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setFixedWidth(105)
        self._btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self._btn_connect)

        self._lbl_status = QLabel("Disconnected")
        self._lbl_status.setFixedWidth(110)
        lay.addWidget(self._lbl_status)

        self._refresh_ports()
        return grp

    def _build_mode_value_group(self) -> QGroupBox:
        grp = QGroupBox("Mode / Value")
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(8)

        lay.addWidget(QLabel("Mode:"))
        self._lbl_mode = QLabel("RPM")
        lay.addWidget(self._lbl_mode)

        self._lbl_value_label = QLabel("RPM:")
        lay.addWidget(self._lbl_value_label)
        self._lbl_value = QLabel(str(self.current_rpm))
        lay.addWidget(self._lbl_value)

        lay.addWidget(QLabel("Shift+5 / Ctrl-5"))
        lay.addStretch()

        btn_rpm = QPushButton("Mode RPM")
        btn_rpm.clicked.connect(lambda: self._set_mode("RPM"))
        lay.addWidget(btn_rpm)

        btn_pwm = QPushButton("Mode DUTY")
        btn_pwm.clicked.connect(lambda: self._set_mode("DUTY"))
        lay.addWidget(btn_pwm)

        return grp

    def _build_operating_mode_group(self) -> QGroupBox:
        grp = QGroupBox("Operating Mode")
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(8)

        self._lbl_op_mode_status = QLabel("DISARM")
        self._lbl_op_mode_status.setAlignment(Qt.AlignCenter)
        self._lbl_op_mode_status.setFixedHeight(30)
        lay.addWidget(self._lbl_op_mode_status, 1)

        for key, cfg in self.OPERATING_MODES.items():
            btn = QPushButton(cfg["label"])
            btn.clicked.connect(lambda _=False, k=key: self._set_operating_mode(k))
            lay.addWidget(btn)

        self._update_operating_mode_ui(self._operating_mode)
        return grp

    def _build_motor_table_group(self) -> QGroupBox:
        grp = QGroupBox("Motor Telemetry")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 4)

        num_cols = len(self.MOTOR_COL_HEADERS)
        self._motor_table = QTableWidget(4, num_cols)
        self._motor_table.setHorizontalHeaderLabels(self.MOTOR_COL_HEADERS)
        headers = self._motor_table.horizontalHeader()
        if headers:
            headers.setStretchLastSection(True)
            headers.setSectionResizeMode(
                self.MOTOR_COL["motor"], QHeaderView.ResizeMode.ResizeToContents)

        motors = ["FL", "FR", "RL", "RR"]
        for row, name in enumerate(motors):
            self._motor_table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, num_cols):
                self._motor_table.setItem(row, col, QTableWidgetItem("--"))

        self._motor_table.setVerticalHeaderLabels([])
        self._motor_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._motor_table.setFocusPolicy(Qt.NoFocus)
        self._motor_table.verticalHeader().setVisible(False)
        self._motor_table.setMaximumHeight(160)
        self._motor_table.setMinimumHeight(120)
        lay.addWidget(self._motor_table)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self._btn_motor_settings = QPushButton("Settings")
        self._btn_motor_settings.clicked.connect(self._open_motor_settings)
        btn_row.addWidget(self._btn_motor_settings)
        lay.addLayout(btn_row)
        return grp

    def _build_console_group(self) -> QGroupBox:
        grp = QGroupBox("Console")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(4)

        self._console = QTextEdit()
        self._console.setReadOnly(True)
        lay.addWidget(self._console)

        input_row = QHBoxLayout()
        input_row.setSpacing(4)

        self._h7_input = QLineEdit()
        self._h7_input.setPlaceholderText("Type H7 command and press Enter...")
        self._h7_input.returnPressed.connect(self._send_h7_input)
        input_row.addWidget(self._h7_input, 1)

        self._btn_h7_send = QPushButton("Send")
        self._btn_h7_send.clicked.connect(self._send_h7_input)
        input_row.addWidget(self._btn_h7_send)

        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(60)
        btn_clear.clicked.connect(self._console.clear)
        input_row.addWidget(btn_clear)

        lay.addLayout(input_row)
        return grp

    # ----------------------------------------------------------------------
    #  Stylesheet
    # ----------------------------------------------------------------------

    def _apply_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background: #111111; }
            QWidget { background: #111111; color: #dddddd; font-size: 12px; }
            QGroupBox {
                border: 1px solid #444444;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }
            QPushButton {
                background: #222222;
                border: 1px solid #555555;
                border-radius: 4px;
                padding: 4px 10px;
                min-height: 22px;
            }
            QPushButton:hover { background: #333333; }
            QPushButton:pressed { background: #444444; }
            QLineEdit, QComboBox {
                background: #181818;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 3px 6px;
                color: #dddddd;
            }
            QComboBox QAbstractItemView {
                background: #181818;
                border: 1px solid #555555;
                selection-background-color: #333333;
                color: #dddddd;
            }
            QTableWidget {
                background: #0a0a0a;
                border: 1px solid #444444;
                gridline-color: #333333;
                color: #dddddd;
            }
            QHeaderView::section {
                background: #1a1a1a;
                color: #cccccc;
                border: none;
                border-right: 1px solid #444444;
                border-bottom: 1px solid #444444;
                padding: 3px;
                font-weight: bold;
            }
            QTextEdit {
                background: #050505;
                border: 1px solid #444444;
                border-radius: 3px;
                color: #cccccc;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
            }
        """)

    # ----------------------------------------------------------------------
    #  Console Logging (single console, max lines)
    # ----------------------------------------------------------------------

    def _console_append(self, tag: str, text: str):
        ts = time.strftime("%H:%M:%S")
        self._console.append(f"[{ts}] [{tag}] {text}")
        if self._console.document().blockCount() > MAX_CONSOLE_LINES:
            cursor = self._console.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _log_tx(self, cmd: str):
        self._console_append("TX", cmd)

    def _log_rx(self, text: str):
        self._console_append("RX", text)

    def _log_info(self, text: str):
        self._console_append("GUI", text)

    def _log_err(self, text: str):
        self._console_append("ERR", text)

    def _log_warn(self, text: str):
        self._console_append("WARN", text)

    # ----------------------------------------------------------------------
    #  Serial Port Management
    # ----------------------------------------------------------------------

    def _refresh_ports(self):
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self._port_combo.addItem(p.device)
        if not self._port_combo.count():
            self._log_info("No serial ports found.")

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_combo.currentText()
        if not port:
            self._log_warn("No port selected.")
            return
        try:
            baud = int(self._baud_edit.text())
        except ValueError:
            self._log_err("Invalid baudrate.")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.connected = True

            self.reader_thread = SerialReaderThread(self.ser)
            self.reader_thread.line_received.connect(self._on_rx_line)
            self.reader_thread.error_occurred.connect(
                lambda e: self._log_err(f"Reader: {e}")
            )
            self.reader_thread.disconnected.connect(self._handle_disconnect)
            self.reader_thread.start()

            self._lbl_status.setText("Connected")
            self._btn_connect.setText("Disconnect")
            self._port_combo.setEnabled(False)
            self._baud_edit.setEnabled(False)
            self._telemetry_timer.start()
            self._log_info(f"Connected to {port} @ {baud}")
        except Exception as e:
            self._log_err(f"Connect failed: {e}")

    def _disconnect(self):
        self._stop_reader()
        self._close_serial()
        self._set_disconnected_ui()
        self._log_info("Disconnected.")

    def _handle_disconnect(self):
        self._close_serial()
        self._set_disconnected_ui()
        self._log_warn("Connection lost.")

    def _stop_reader(self):
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread = None

    def _close_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False

    def _set_disconnected_ui(self):
        self.connected = False
        self._pending_mode = None
        if self._pending_mode_timer.isActive():
            self._pending_mode_timer.stop()
        if self._tuning_send_timer.isActive():
            self._tuning_send_timer.stop()
            self._tuning_send_queue.clear()
            if self._tuning_dialog_ref is not None:
                try:
                    self._tuning_dialog_ref._set_send_buttons_enabled(True)
                except Exception:
                    pass
                self._tuning_dialog_ref = None
        self._telemetry_timer.stop()
        self._lbl_status.setText("Disconnected")
        self._btn_connect.setText("Connect")
        self._port_combo.setEnabled(True)
        self._baud_edit.setEnabled(True)

    # ----------------------------------------------------------------------
    #  Serial Receive
    # ----------------------------------------------------------------------

    def _on_rx_line(self, line: str):
        self._log_rx(line)
        if self._parse_motor_telemetry_line(line):
            pass
        else:
            self._parse_rx_for_motor_state(line)
        self._parse_uart_error_line(line)
        self._parse_operating_mode_confirm(line)

    def _parse_rx_for_motor_state(self, line: str):
        lower = line.lower()
        link_col = self.MOTOR_COL["link"]
        for tag, row in self.MOTOR_ROW.items():
            if f"link_lost][{tag}" in lower:
                item = self._motor_table.item(row, link_col)
                if item:
                    item.setText("LOST")
            if f"link_recovered][{tag}" in lower:
                item = self._motor_table.item(row, link_col)
                if item:
                    item.setText("OK")

    # ── UART error parsing ───────────────────────────────────────────────

    def _set_motor_error(self, motor: str, text: str, is_error: bool):
        row = self.MOTOR_ROW.get(motor)
        if row is None:
            return
        self._motor_uart_error_text[motor] = text if is_error else ""
        self._render_motor_error(motor)

    def _render_motor_error(self, motor: str):
        row = self.MOTOR_ROW.get(motor)
        if row is None:
            return
        col = self.MOTOR_COL["error"]
        item = self._motor_table.item(row, col)
        if item is None:
            return
        uart_err = self._motor_uart_error_text.get(motor, "")
        fc = self._motor_fault_code.get(motor, "0")
        if uart_err:
            item.setText(uart_err)
        elif fc != "0":
            item.setText(f"FC:{fc}")
        else:
            item.setText("--")

    def _parse_uart_error_line(self, line: str) -> bool:
        m = _RE_UART_ERROR_CODE.match(line)
        if m:
            uart, code = m.group(1), m.group(2)
            motor = self.UART_TO_MOTOR.get(uart)
            if motor is not None:
                self._uart_report_decoded[uart] = []
                self._set_motor_error(motor, f"UART err: {code}", is_error=True)
            return True

        m = _RE_UART_ERROR_DECODED.match(line)
        if m:
            uart, desc = m.group(1), m.group(2).strip()
            motor = self.UART_TO_MOTOR.get(uart)
            if motor is not None:
                code = desc.split(" - ", 1)[0].strip().upper()
                if code in self.UART_ERROR_CODES:
                    buf = self._uart_report_decoded.setdefault(uart, [])
                    if desc not in buf:
                        buf.append(desc)
                    self._set_motor_error(motor, ", ".join(buf), is_error=True)
            return True

        m = _RE_UART_RECOVERED.match(line)
        if m:
            uart = m.group(1)
            motor = self.UART_TO_MOTOR.get(uart)
            if motor is not None:
                self._uart_report_decoded.pop(uart, None)
                self._set_motor_error(motor, "", is_error=False)
            return True

        return False

    # ── Motor telemetry parsing (buffered) ──────────────────────────────

    def _parse_motor_telemetry_line(self, line: str) -> bool:
        motor = None
        payload = None

        m = _RE_MOTOR_TEL_TAGGED.match(line)
        if m:
            motor = m.group(1)
            payload = m.group(2)
        else:
            m = _RE_MOTOR_TEL_UART.match(line)
            if m:
                motor = self.UART_RX_TO_MOTOR.get(m.group(1))
                payload = m.group(2)

        if motor is None or payload is None:
            return False

        tel = self._parse_telemetry_payload(payload)
        if not tel:
            return False

        self._motor_telemetry[motor].update(tel)
        self._telemetry_buffer[motor].update(tel)
        self._telemetry_dirty = True
        return True

    @staticmethod
    def _parse_telemetry_payload(payload: str) -> dict[str, str]:
        result = {}
        for token in payload.split(","):
            if ":" not in token:
                continue
            key, val = token.split(":", 1)
            result[key.strip()] = val.strip()
        return result

    def _set_table_text_if_changed(self, row: int, col: int, text: str):
        item = self._motor_table.item(row, col)
        if item is not None and item.text() != text:
            item.setText(text)

    def _flush_telemetry_table(self):
        if not self._telemetry_dirty:
            return
        self._telemetry_dirty = False

        for motor, tel in self._telemetry_buffer.items():
            if not tel:
                continue
            row = self.MOTOR_ROW.get(motor)
            if row is None:
                continue
            col = self.MOTOR_COL

            self._set_table_text_if_changed(row, col["current_rpm"], tel.get("RPM", "--"))
            self._set_table_text_if_changed(row, col["target_rpm"], tel.get("T", "--"))
            self._set_table_text_if_changed(row, col["pwm"], tel.get("PWM_ACT", tel.get("PWM_SET", "--")))

            dir_val = tel.get("DIR", "")
            if dir_val:
                self._set_table_text_if_changed(row, col["direction"], self._DIR_MAP.get(dir_val, dir_val))

            sp = tel.get("SP", "")
            if sp:
                self._set_table_text_if_changed(row, col["control_mode"], self._SP_MAP.get(sp, sp))

            brk = tel.get("BRAKE", "")
            if brk:
                self._set_table_text_if_changed(row, col["brake_status"], self._BRAKE_MAP.get(brk, brk))

            fc = tel.get("FC", "")
            if fc:
                self._motor_fault_code[motor] = fc
                self._set_table_text_if_changed(row, col["fault_code"], "--" if fc == "0" else f"FC:{fc}")

            self._render_motor_error(motor)

            link_item = self._motor_table.item(row, col["link"])
            if link_item is not None and link_item.text() != "OK":
                link_item.setText("OK")

            self._telemetry_buffer[motor].clear()

    # ── Operating mode confirmation parsing ─────────────────────────────

    _OP_MODE_CONFIRM_TO_KEY = {
        "DISARM": "disarm",
        "MANUAL": "manual",
        "AUTONOMOUS": "auto",
    }

    def _parse_operating_mode_confirm(self, line: str) -> bool:
        m = _RE_OP_MODE_CONFIRM.search(line)
        if not m:
            return False
        mode_name = m.group(1)
        mode_key = self._OP_MODE_CONFIRM_TO_KEY.get(mode_name)
        if mode_key is None:
            return True
        was_pending = self._pending_mode
        self._pending_mode = None
        self._pending_mode_timer.stop()
        self._update_operating_mode_ui(mode_key)
        if was_pending == mode_key:
            self._log_info(f"Mode confirmed: {mode_name}")
        elif was_pending is not None:
            self._log_warn(f"H7 confirmed {mode_name} (expected {was_pending})")
        else:
            self._log_info(f"Operating mode: {mode_name}")
        return True

    def _on_pending_mode_timeout(self):
        failed = self._pending_mode
        self._pending_mode = None
        if failed is None:
            return
        cur_label = self.OPERATING_MODES.get(self._operating_mode, {}).get("label", self._operating_mode)
        self._log_warn(f"Mode change to {failed} not confirmed by H7 — keeping {cur_label}")

    # ----------------------------------------------------------------------
    #  Serial Send
    # ----------------------------------------------------------------------

    def _send_cmd(self, cmd: str):
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log_warn("Not connected.")
            return
        try:
            self.ser.write((cmd + "\r\n").encode("utf-8"))
            self._log_tx(cmd)
        except Exception as e:
            self._log_err(f"Send failed: {e}")

    def _send_h7_input(self):
        text = self._h7_input.text().strip()
        if not text:
            return
        self._h7_input.clear()
        if not self.connected:
            self._log_warn("Cannot send: not connected.")
        else:
            self._send_cmd(text)
        self.setFocus()

    # ----------------------------------------------------------------------
    #  Mode / Value Management
    # ----------------------------------------------------------------------

    def _get_current_value(self) -> int:
        return self.current_rpm if self.mode == "RPM" else self.current_pwm

    def _set_operating_mode(self, mode_key: str):
        cfg = self.OPERATING_MODES.get(mode_key)
        if cfg is None:
            return
        self._send_cmd(cfg["command"])
        already_pending = self._pending_mode is not None
        self._pending_mode = mode_key
        self._pending_mode_timer.start()
        if not already_pending:
            self._log_info(f"Requested {cfg['label']} — waiting for H7 confirmation...")

    def _update_operating_mode_ui(self, mode_key: str):
        cfg = self.OPERATING_MODES.get(mode_key)
        if cfg is None:
            return
        self._operating_mode = mode_key
        self._lbl_op_mode_status.setText(cfg["label"])
        self._lbl_op_mode_status.setStyleSheet(
            f"background-color: {cfg['status_bg']}; color: {cfg['status_fg']}; "
            f"font-size: 14px; font-weight: bold; border-radius: 4px;"
        )

    def _set_mode(self, new_mode: str):
        if new_mode == self.mode:
            return
        self.mode = new_mode
        self._lbl_mode.setText(new_mode)
        if new_mode == "RPM":
            self._lbl_value_label.setText("RPM:")
            self._lbl_value.setText(str(self.current_rpm))
            self._send_cmd("m speed")
        else:
            self._lbl_value_label.setText("Duty:")
            self._lbl_value.setText(str(self.current_pwm))
            self._send_cmd("m duty")
        self._log_info(f"Mode: {new_mode}")

    def _toggle_mode(self):
        self._set_mode("DUTY" if self.mode == "RPM" else "RPM")

    def _adjust_value(self, delta: int):
        if self.mode == "RPM":
            self.current_rpm = max(0, min(self.RPM_MAX, self.current_rpm + delta))
            self._lbl_value.setText(str(self.current_rpm))
        else:
            self.current_pwm = max(0, min(self.PWM_MAX, self.current_pwm + delta))
            self._lbl_value.setText(str(self.current_pwm))

    # ----------------------------------------------------------------------
    #  Movement Command Mapping
    # ----------------------------------------------------------------------

    def _movement_cmd(self, key: str) -> str:
        val = self._get_current_value()
        if self.mode == "RPM":
            return {"W": f"f{val}", "S": f"b{val}", "A": f"l{val}", "D": f"r{val}"}[key]
        else:
            return {"W": f"fd{val}", "S": f"bd{val}", "A": f"ld{val}", "D": f"rd{val}"}[key]

    # ----------------------------------------------------------------------
    #  Keyboard Handling
    # ----------------------------------------------------------------------

    def eventFilter(self, obj, event):
        if obj is self._h7_input:
            if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Escape:
                self.setFocus()
                return True
            return False
        return super().eventFilter(obj, event)

    def _key_to_id(self, event) -> str | None:
        key = event.key()
        text = event.text().upper()
        if text in self.MOVEMENT_KEYS:
            return text
        if key == Qt.Key_Space:
            return "Space"
        if text == "X":
            return "X"
        if text == "M":
            return "M"
        if key == Qt.Key_Shift:
            return "Shift"
        if key == Qt.Key_Control:
            return "Ctrl"
        return None

    def keyPressEvent(self, event: QKeyEvent):
        key_id = self._key_to_id(event)
        if not key_id or key_id in self._keys_held:
            super().keyPressEvent(event)
            return

        self._keys_held.add(key_id)

        if key_id in self.MOVEMENT_KEYS:
            self._move_held.add(key_id)
            self._move_order.append(key_id)
            self._active_move_key = self._move_order[-1]
            self._send_cmd(self._movement_cmd(self._active_move_key))
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "Space":
            self._send_cmd("stop")
        elif key_id == "X":
            self._send_cmd("brake")
        elif key_id == "M":
            self._toggle_mode()
        elif key_id == "Shift":
            self._active_modifier = "Shift"
            self._adjust_value(self.VALUE_STEP)
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "Ctrl":
            self._active_modifier = "Ctrl"
            self._adjust_value(-self.VALUE_STEP)
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()

    def keyReleaseEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key_id = self._key_to_id(event)
        if not key_id or key_id not in self._keys_held:
            super().keyReleaseEvent(event)
            return

        self._keys_held.discard(key_id)

        if key_id in self.MOVEMENT_KEYS:
            self._move_held.discard(key_id)
            self._move_order = deque(k for k in self._move_order if k != key_id)
            if self._move_order:
                self._active_move_key = self._move_order[-1]
            else:
                self._active_move_key = None
                if not self._active_modifier:
                    self._repeat_timer.stop()
                self._send_cmd("stop")
        elif key_id in ("Shift", "Ctrl"):
            self._active_modifier = None
            if not self._active_move_key:
                self._repeat_timer.stop()

        super().keyReleaseEvent(event)

    def _repeat_movement(self):
        if self._active_move_key:
            self._send_cmd(self._movement_cmd(self._active_move_key))
        if self._active_modifier == "Shift":
            self._adjust_value(self.VALUE_STEP)
        elif self._active_modifier == "Ctrl":
            self._adjust_value(-self.VALUE_STEP)
        if not self._active_move_key and not self._active_modifier:
            self._repeat_timer.stop()

    # ----------------------------------------------------------------------
    #  F411 Motor Tuning
    # ----------------------------------------------------------------------

    F411_TUNE_KW_BASE     = "base"
    F411_TUNE_KW_BOOST    = "boost"
    F411_TUNE_KW_KICKDUTY = "kickduty"
    F411_TUNE_KW_KICKMS   = "kickms"
    F411_TUNE_KW_RAMP     = "ramp"
    F411_TUNE_KW_PI       = "pi"
    F411_TUNE_KW_TELPER   = "telper"

    def build_f411_tuning_commands(self, target_motor: str, settings: dict) -> list:
        cmds = []
        log = self._log_warn

        bases = settings.get("base", [""] * 8) or [""] * 8
        while len(bases) < 8:
            bases.append("")
        all_empty = all(v == "" for v in bases[:8])
        some_empty = not all_empty and any(v == "" for v in bases[:8])
        if not all_empty:
            if some_empty:
                log("[F411-TUNE] Base PWM: not all 8 values filled — skipped")
            else:
                vals = " ".join(v if v != "" else "0" for v in bases[:8])
                cmds.append(f"{target_motor} {self.F411_TUNE_KW_BASE} {vals}")

        boosts = settings.get("boost", [""] * 8) or [""] * 8
        while len(boosts) < 8:
            boosts.append("")
        boostms = settings.get("boostms", "")
        all_b_empty = all(v == "" for v in boosts[:8]) and boostms == ""
        some_b_empty = not all_b_empty and (
            any(v == "" for v in boosts[:8]) or boostms == "")
        if not all_b_empty:
            if some_b_empty:
                log("[F411-TUNE] Boost: need all 8 PWM + MS — skipped")
            else:
                pvals = " ".join(v if v != "" else "0" for v in boosts[:8])
                cmds.append(f"{target_motor} {self.F411_TUNE_KW_BOOST} {pvals} {boostms}")

        kick_duty = settings.get("kick_duty", "")
        if kick_duty:
            cmds.append(f"{target_motor} {self.F411_TUNE_KW_KICKDUTY} {kick_duty}")

        kick_ms = settings.get("kick_ms", "")
        if kick_ms:
            cmds.append(f"{target_motor} {self.F411_TUNE_KW_KICKMS} {kick_ms}")

        ramp_up = settings.get("ramp_up", "")
        ramp_dn = settings.get("ramp_down", "")
        if ramp_up and ramp_dn:
            cmds.append(f"{target_motor} {self.F411_TUNE_KW_RAMP} {ramp_up} {ramp_dn}")
        elif ramp_up or ramp_dn:
            log("[F411-TUNE] Ramp: need both Up and Down — skipped")

        kp = settings.get("kp", "")
        ki = settings.get("ki", "")
        if kp and ki:
            cmds.append(f"{target_motor} {self.F411_TUNE_KW_PI} {kp} {ki}")
        elif kp or ki:
            log("[F411-TUNE] PI: need both Kp and Ki — skipped")

        telper = settings.get("telper", "")
        if telper:
            cmds.append(f"{target_motor} {self.F411_TUNE_KW_TELPER} {telper}")

        return cmds

    def send_f411_tuning_command(self, target_motor: str, command: str):
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log_warn(f"[F411-TUNE] Not sent (disconnected): {command}")
            return
        self._log_info(f"[F411-TUNE] {command}")
        self._send_cmd(command)

    def _open_motor_settings(self):
        dlg = MotorSettingsDialog(self, self)
        dlg.exec()

    def enqueue_f411_tuning_sequence(self, commands: list[str], dialog=None):
        if self._tuning_send_queue:
            self._log_warn("[F411-TUNE] Sequence already running")
            return
        self._tuning_dialog_ref = dialog
        self._tuning_send_queue.extend(commands)
        self._log_info(f"[F411-TUNE] Queued {len(commands)} command(s)")
        if dialog is not None:
            dialog._set_send_buttons_enabled(False)
        self._send_next_f411_tuning_command()
        if self._tuning_send_queue:
            self._tuning_send_timer.start()

    def _send_next_f411_tuning_command(self):
        if not self._tuning_send_queue:
            self._tuning_send_timer.stop()
            self._log_info("[F411-TUNE] Sequence complete")
            dlg = self._tuning_dialog_ref
            self._tuning_dialog_ref = None
            if dlg is not None:
                dlg._set_send_buttons_enabled(True)
            return

        cmd = self._tuning_send_queue.popleft()
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log_warn(f"[F411-TUNE] Aborted (disconnected): {cmd}")
            self._tuning_send_queue.clear()
            self._tuning_send_timer.stop()
            dlg = self._tuning_dialog_ref
            self._tuning_dialog_ref = None
            if dlg is not None:
                dlg._set_send_buttons_enabled(True)
            return

        self._send_cmd(cmd)

    # ----------------------------------------------------------------------
    #  Cleanup
    # ----------------------------------------------------------------------

    def closeEvent(self, event):
        self._tuning_send_timer.stop()
        self._tuning_send_queue.clear()
        self._telemetry_timer.stop()
        self._repeat_timer.stop()
        self._stop_reader()
        self._close_serial()
        super().closeEvent(event)


# ----------------------------------------------------------------------------
#  Entry Point
# ----------------------------------------------------------------------------

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = EarendilControlGui()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
