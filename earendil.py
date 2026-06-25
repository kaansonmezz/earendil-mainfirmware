#!/usr/bin/env python3
"""
Earendil — Rover Control GUI for STM32H723ZG Main Controller
============================================================
Single-file PySide6 + pyserial application.
Communicates with the H7 firmware over USART3 / ST-LINK VCP.

Controls:
    W/A/S/D   -> forward / left / backward / right
    Space     -> stop
    X         -> brake
    M         -> toggle mode (RPM / PWM)
    I         -> identify
    LShift    -> increase RPM/PWM by +5
    LCtrl     -> decrease RPM/PWM by -5

Run:
    python earendil.py
"""

import sys
import time
import threading
from collections import deque

# ── Dependency check ───────────────────────────────────────────────────────
_missing = []
try:
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QPushButton, QComboBox, QLineEdit,
        QGroupBox, QTextEdit, QTableWidget, QTableWidgetItem,
        QDialog, QHeaderView, QSplitter, QFrame, QSizePolicy,
    )
    from PySide6.QtCore import Qt, QTimer, Signal, QObject, QThread, QEvent
    from PySide6.QtGui import QFont, QColor, QTextCursor, QKeyEvent, QKeySequence
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


# ════════════════════════════════════════════════════════════════════════════
#  Serial Reader Thread
# ════════════════════════════════════════════════════════════════════════════

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


# ════════════════════════════════════════════════════════════════════════════
#  Main GUI
# ════════════════════════════════════════════════════════════════════════════

class EarendilControlGui(QMainWindow):
    """
    Main rover control window.
    Left side: control panels.  Right side: serial console.
    """

    # ── Styling ────────────────────────────────────────────────────────────
    STYLE = """
    QMainWindow {
        background-color: #101014;
    }
    QWidget {
        color: #C0C0C0;
        font-size: 13px;
    }
    QGroupBox {
        border: 1px solid #5F5A4A;
        border-radius: 6px;
        margin-top: 10px;
        padding-top: 14px;
        font-weight: bold;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 12px;
        padding: 0 6px;
        color: #D4AF37;
    }
    QPushButton {
        background-color: #2A2A31;
        border: 1px solid #5F5A4A;
        border-radius: 6px;
        padding: 6px 14px;
        min-height: 28px;
        color: #C0C0C0;
    }
    QPushButton:hover {
        background-color: #3A3320;
        border-color: #8A6F2A;
    }
    QPushButton:pressed {
        background-color: #4A4230;
    }
    QComboBox, QLineEdit {
        background-color: #2A2A31;
        border: 1px solid #5F5A4A;
        border-radius: 4px;
        padding: 4px 8px;
        color: #C0C0C0;
    }
    QComboBox QAbstractItemView {
        background-color: #2A2A31;
        border: 1px solid #5F5A4A;
        selection-background-color: #3A3320;
        color: #C0C0C0;
    }
    QTableWidget {
        background-color: #0B0B0D;
        border: 1px solid #5F5A4A;
        border-radius: 4px;
        gridline-color: #2A2A31;
        selection-background-color: #3A3320;
        color: #C0C0C0;
    }
    QTableWidget::item {
        padding: 4px;
    }
    QHeaderView::section {
        background-color: #1E1E24;
        color: #D4AF37;
        border: none;
        border-right: 1px solid #5F5A4A;
        border-bottom: 1px solid #5F5A4A;
        padding: 4px;
        font-weight: bold;
    }
    QTextEdit {
        background-color: #0B0B0D;
        border: 1px solid #5F5A4A;
        border-radius: 4px;
        color: #C0C0C0;
        font-family: 'Consolas', 'Courier New', monospace;
        font-size: 12px;
    }
    QSplitter::handle {
        background-color: #5F5A4A;
        width: 3px;
    }
    """

    # ── Constants ──────────────────────────────────────────────────────────
    REPEAT_INTERVAL_MS = 500
    DEFAULT_RPM = 100
    DEFAULT_PWM = 100
    RPM_MAX = 200
    PWM_MAX = 255
    VALUE_STEP = 5

    # Movement key priority: most-recently-pressed wins
    MOVE_KEYS = {
        Qt.Key_W: "W",
        Qt.Key_S: "S",
        Qt.Key_A: "A",
        Qt.Key_D: "D",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Earendil — Rover Control")
        self.setMinimumSize(1100, 650)
        self.setStyleSheet(self.STYLE)

        # ── State ──────────────────────────────────────────────────────────
        self.ser: serial.Serial | None = None
        self.reader_thread: SerialReaderThread | None = None
        self.connected = False

        self.mode = "RPM"               # "RPM" or "PWM"
        self.current_rpm = self.DEFAULT_RPM
        self.current_pwm = self.DEFAULT_PWM

        self._active_move_key: str | None = None   # current movement key (W/A/S/D)
        self._move_held: set[str] = set()          # held movement keys
        self._move_order: deque[str] = deque()     # movement key press order
        self._active_modifier: str | None = None   # "Shift" or "Ctrl" if held
        self._keys_held: set[str] = set()          # ALL held keys (prevents duplicates)

        # ── Build UI ───────────────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        # ── Top row: Serial Connection (left) + Rover Status (right) ───
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(8)
        top_row.addWidget(self._build_connection_group())
        top_row.addWidget(self._build_rover_status_group(), 1)

        left_layout.addLayout(top_row)
        left_layout.addWidget(self._build_mode_value_group())
        left_layout.addWidget(self._build_motor_table_group())
        left_layout.addStretch()

        splitter.addWidget(left_panel)

        # Right panel — console
        splitter.addWidget(self._build_console_group())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # ── Repeat timer ───────────────────────────────────────────────────
        self._repeat_timer = QTimer(self)
        self._repeat_timer.setInterval(self.REPEAT_INTERVAL_MS)
        self._repeat_timer.timeout.connect(self._repeat_movement)

        # Prevent buttons from stealing keyboard focus (Space must always reach keyPressEvent)
        for btn in self.findChildren(QPushButton):
            btn.setFocusPolicy(Qt.NoFocus)

        # H7 input handles its own keyboard events; main window handles the rest
        self._h7_input.installEventFilter(self)
        self.setFocusPolicy(Qt.StrongFocus)

        self._log_info("Ready. Connect to rover to begin.")

    # ══════════════════════════════════════════════════════════════════════
    #  UI Builders
    # ══════════════════════════════════════════════════════════════════════

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("Serial Connection")
        grp.setMaximumWidth(800)
        grp.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
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
        self._btn_connect.setStyleSheet("QPushButton { background-color: #1e6e3e; color: #C0C0C0; }")
        self._btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self._btn_connect)

        self._lbl_status = QLabel("● Disconnected")
        self._lbl_status.setStyleSheet("color: #B00020; font-weight: bold;")
        self._lbl_status.setFixedWidth(120)
        lay.addWidget(self._lbl_status)

        self._refresh_ports()
        return grp

    def _build_rover_status_group(self) -> QGroupBox:
        grp = QGroupBox("Rover Status")
        grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(16)

        def _badge(initial: str, color: str) -> QLabel:
            lbl = QLabel(initial)
            lbl.setStyleSheet(
                f"color: {color}; font-weight: bold; "
                "background-color: #0B0B0D; border: 1px solid #5F5A4A; "
                "border-radius: 4px; padding: 4px 10px;"
            )
            return lbl

        self._lbl_qs_mode = _badge("Mode: RPM", "#D4AF37")
        lay.addWidget(self._lbl_qs_mode)

        self._lbl_qs_motion = _badge("Motion: IDLE", "#5F5A4A")
        lay.addWidget(self._lbl_qs_motion)

        self._lbl_qs_port = _badge("Port: Disconnected", "#B00020")
        lay.addWidget(self._lbl_qs_port)

        lay.addStretch()
        return grp

    def _build_mode_value_group(self) -> QGroupBox:
        grp = QGroupBox("Mode / Value")
        lay = QHBoxLayout(grp)

        lay.addWidget(QLabel("Mode:"))
        self._lbl_mode = QLabel("RPM")
        self._lbl_mode.setStyleSheet(
            "color: #D4AF37; font-size: 16px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_mode)

        lay.addSpacing(20)
        self._lbl_value_label = QLabel("RPM Value:")
        lay.addWidget(self._lbl_value_label)
        self._lbl_value = QLabel(str(self.current_rpm))
        self._lbl_value.setStyleSheet(
            "color: #FFD66B; font-size: 18px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_value)

        lay.addSpacing(10)
        lay.addWidget(QLabel("Shift +5 / Ctrl -5"))

        lay.addStretch()

        btn_rpm = QPushButton("Mode RPM")
        btn_rpm.clicked.connect(lambda: self._set_mode("RPM"))
        lay.addWidget(btn_rpm)

        btn_pwm = QPushButton("Mode PWM")
        btn_pwm.clicked.connect(lambda: self._set_mode("PWM"))
        lay.addWidget(btn_pwm)

        self._btn_help = QPushButton("GUI Help")
        self._btn_help.setStyleSheet(
            "QPushButton { background-color: #2A2A31; border: 1px solid #D4AF37; "
            "color: #D4AF37; font-weight: bold; }"
            "QPushButton:hover { background-color: #3A3320; }"
        )
        self._btn_help.clicked.connect(self._show_help_popup)
        lay.addWidget(self._btn_help)

        return grp

    _DIR_STYLE_ACTIVE = (
        "color: #FFD66B; font-size: 13px; font-weight: bold; "
        "background-color: #0B0B0D; border: 1px solid #D4AF37; "
        "border-radius: 4px; padding: 2px 8px;"
    )
    _DIR_STYLE_IDLE = (
        "color: #5F5A4A; font-size: 13px; font-weight: bold; "
        "background-color: #0B0B0D; border: 1px solid #3A3A3A; "
        "border-radius: 4px; padding: 2px 8px;"
    )

    def _build_motor_table_group(self) -> QGroupBox:
        grp = QGroupBox("Motor State")
        lay = QVBoxLayout(grp)

        # ── Compact motion indicator (top-left inside Motor State) ────
        motion_row = QHBoxLayout()
        self._lbl_direction = QLabel("IDLE")
        self._lbl_direction.setAlignment(Qt.AlignCenter)
        self._lbl_direction.setFixedWidth(110)
        self._lbl_direction.setStyleSheet(self._DIR_STYLE_IDLE)
        motion_row.addWidget(self._lbl_direction)
        motion_row.addStretch()
        lay.addLayout(motion_row)

        self._motor_table = QTableWidget(4, 7)
        self._motor_table.setHorizontalHeaderLabels(
            ["Motor", "Mode", "PWM", "RPM", "Error", "Link", "Last RX"]
        )
        headers = self._motor_table.horizontalHeader()
        if headers:
            headers.setSectionResizeMode(QHeaderView.Stretch)

        motors = ["FL", "FR", "RL", "RR"]
        for row, name in enumerate(motors):
            self._motor_table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 7):
                self._motor_table.setItem(row, col, QTableWidgetItem("--"))

        self._motor_table.setVerticalHeaderLabels([])
        self._motor_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._motor_table.setMaximumHeight(160)
        lay.addWidget(self._motor_table)
        return grp

    def _update_motion_indicator(self, direction: str | None):
        """Update the Motion box.  direction is one of W/S/A/D or None for IDLE."""
        mapping = {"W": "FORWARD", "S": "BACKWARD", "A": "LEFT", "D": "RIGHT"}
        text = mapping.get(direction, "IDLE")
        self._lbl_direction.setText(text)
        self._lbl_direction.setStyleSheet(
            self._DIR_STYLE_ACTIVE if text != "IDLE" else self._DIR_STYLE_IDLE
        )
        color = "#FFD66B" if text != "IDLE" else "#5F5A4A"
        self._lbl_qs_motion.setText(f"Motion: {text}")
        self._lbl_qs_motion.setStyleSheet(
            f"color: {color}; font-weight: bold; "
            "background-color: #0B0B0D; border: 1px solid #5F5A4A; "
            "border-radius: 4px; padding: 4px 10px;"
        )

    def _build_console_group(self) -> QGroupBox:
        grp = QGroupBox("Console")
        lay = QVBoxLayout(grp)

        console_splitter = QSplitter(Qt.Vertical)

        # ── H7 Console ────────────────────────────────────────────────
        h7_widget = QWidget()
        h7_lay = QVBoxLayout(h7_widget)
        h7_lay.setContentsMargins(0, 0, 0, 0)
        h7_lay.setSpacing(4)

        h7_label = QLabel("H7 Console")
        h7_label.setStyleSheet(
            "color: #D4AF37; font-weight: bold; font-size: 13px;"
        )
        h7_lay.addWidget(h7_label)

        self._h7_console = QTextEdit()
        self._h7_console.setReadOnly(True)
        self._h7_console.setStyleSheet(
            "QTextEdit { background-color: #0B0B0D; border: 1px solid #D4AF37; "
            "border-radius: 4px; color: #C0C0C0; "
            "font-family: 'Consolas', 'Courier New', monospace; font-size: 12px; }"
        )
        h7_lay.addWidget(self._h7_console)

        h7_input_lay = QHBoxLayout()
        h7_input_lay.setContentsMargins(0, 0, 0, 0)
        h7_input_lay.setSpacing(4)

        self._h7_input = QLineEdit()
        self._h7_input.setPlaceholderText("Type H7 command and press Enter...")
        self._h7_input.setStyleSheet(
            "QLineEdit { background-color: #2A2A31; border: 1px solid #D4AF37; "
            "border-radius: 4px; padding: 4px 8px; color: #C0C0C0; }"
        )
        self._h7_input.returnPressed.connect(self._send_h7_input)
        h7_input_lay.addWidget(self._h7_input)

        self._btn_h7_send = QPushButton("Send")
        self._btn_h7_send.setStyleSheet(
            "QPushButton { background-color: #2A2A31; border: 1px solid #D4AF37; "
            "border-radius: 6px; padding: 4px 14px; color: #D4AF37; font-weight: bold; }"
            "QPushButton:hover { background-color: #3A3320; }"
        )
        self._btn_h7_send.clicked.connect(self._send_h7_input)
        h7_input_lay.addWidget(self._btn_h7_send)

        h7_lay.addLayout(h7_input_lay)
        console_splitter.addWidget(h7_widget)

        # ── GUI Console ───────────────────────────────────────────────
        gui_widget = QWidget()
        gui_lay = QVBoxLayout(gui_widget)
        gui_lay.setContentsMargins(0, 0, 0, 0)
        gui_lay.setSpacing(4)

        gui_label = QLabel("GUI Console")
        gui_label.setStyleSheet(
            "color: #8E8E93; font-weight: bold; font-size: 13px;"
        )
        gui_lay.addWidget(gui_label)

        self._gui_console = QTextEdit()
        self._gui_console.setReadOnly(True)
        self._gui_console.setStyleSheet(
            "QTextEdit { background-color: #0B0B0D; border: 1px solid #5F5A4A; "
            "border-radius: 4px; color: #8E8E93; "
            "font-family: 'Consolas', 'Courier New', monospace; font-size: 12px; }"
        )
        gui_lay.addWidget(self._gui_console)

        btn_clear_gui = QPushButton("Clear GUI Console")
        btn_clear_gui.clicked.connect(self._gui_console.clear)
        gui_lay.addWidget(btn_clear_gui)

        console_splitter.addWidget(gui_widget)
        console_splitter.setStretchFactor(0, 3)
        console_splitter.setStretchFactor(1, 2)

        lay.addWidget(console_splitter)

        btn_clear_h7 = QPushButton("Clear H7 Console")
        btn_clear_h7.clicked.connect(self._h7_console.clear)
        lay.addWidget(btn_clear_h7)

        return grp

    # ══════════════════════════════════════════════════════════════════════
    #  Console Logging
    # ══════════════════════════════════════════════════════════════════════

    def _log_h7(self, prefix: str, text: str, color: str = "#C0C0C0"):
        """Append a colored line to the H7 Console."""
        ts = time.strftime("%H:%M:%S")
        self._h7_console.append(
            f"<span style='color:#D4AF37;'>[{ts}]</span> "
            f"<span style='color:{color};'>{prefix} {text}</span>"
        )
        self._h7_console.moveCursor(QTextCursor.End)

    def _log_gui(self, prefix: str, text: str, color: str = "#8E8E93"):
        """Append a colored line to the GUI Console."""
        ts = time.strftime("%H:%M:%S")
        self._gui_console.append(
            f"<span style='color:#D4AF37;'>[{ts}]</span> "
            f"<span style='color:{color};'>{prefix} {text}</span>"
        )
        self._gui_console.moveCursor(QTextCursor.End)

    def _log_tx(self, cmd: str):
        self._log_h7("[TX-H7]", cmd, "#FFD66B")

    def _log_rx(self, text: str):
        self._log_h7("[RX-H7]", text, "#C0C0C0")

    def _log_info(self, text: str):
        self._log_gui("[GUI]", text, "#8E8E93")

    def _log_err(self, text: str):
        self._log_gui("[GUI-ERROR]", text, "#B00020")

    def _log_warn(self, text: str):
        self._log_gui("[GUI-WARN]", text, "#C9831A")

    # ══════════════════════════════════════════════════════════════════════
    #  Serial Port Management
    # ══════════════════════════════════════════════════════════════════════

    def _refresh_ports(self):
        self._port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self._port_combo.addItem(p.device)
        if not ports:
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

            self._lbl_status.setText("● Connected")
            self._lbl_status.setStyleSheet(
                "color: #D4AF37; font-weight: bold;"
            )
            self._btn_connect.setText("Disconnect")
            self._btn_connect.setStyleSheet(
                "QPushButton { background-color: #B00020; color: #C0C0C0; }"
            )
            self._port_combo.setEnabled(False)
            self._baud_edit.setEnabled(False)

            self._log_info(f"Connected to {port} @ {baud}")
            self._lbl_qs_port.setText(f"Port: {port}")
            self._lbl_qs_port.setStyleSheet(
                "color: #D4AF37; font-weight: bold; "
                "background-color: #0B0B0D; border: 1px solid #5F5A4A; "
                "border-radius: 4px; padding: 4px 10px;"
            )
        except Exception as e:
            self._log_err(f"Connect failed: {e}")

    def _disconnect(self):
        self._stop_reader()
        self._close_serial()
        self._set_disconnected_ui()
        self._log_info("Disconnected.")

    def _handle_disconnect(self):
        """Called from reader thread on unexpected disconnect."""
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
        self._lbl_status.setText("● Disconnected")
        self._lbl_status.setStyleSheet(
            "color: #B00020; font-weight: bold;"
        )
        self._btn_connect.setText("Connect")
        self._btn_connect.setStyleSheet(
            "QPushButton { background-color: #1e6e3e; color: #C0C0C0; }"
        )
        self._port_combo.setEnabled(True)
        self._baud_edit.setEnabled(True)
        self._lbl_qs_port.setText("Port: Disconnected")
        self._lbl_qs_port.setStyleSheet(
            "color: #B00020; font-weight: bold; "
            "background-color: #0B0B0D; border: 1px solid #5F5A4A; "
            "border-radius: 4px; padding: 4px 10px;"
        )

    # ══════════════════════════════════════════════════════════════════════
    #  Serial Receive
    # ══════════════════════════════════════════════════════════════════════

    def _on_rx_line(self, line: str):
        self._log_rx(line)
        self._parse_rx_for_motor_state(line)

    def _parse_rx_for_motor_state(self, line: str):
        """Minimal parsing: update Link column if link-lost/recovered detected."""
        lower = line.lower()
        motor_map = {"fl": 0, "fr": 1, "rl": 2, "rr": 3}
        for tag, row in motor_map.items():
            if f"link_lost][{tag}" in lower or f"link lost.*{tag}" in lower:
                item = self._motor_table.item(row, 5)
                if item:
                    item.setText("LOST")
                    item.setForeground(QColor("#B00020"))
            if f"link_recovered][{tag}" in lower:
                item = self._motor_table.item(row, 5)
                if item:
                    item.setText("OK")
                    item.setForeground(QColor("#D4AF37"))

    # ══════════════════════════════════════════════════════════════════════
    #  Serial Send
    # ══════════════════════════════════════════════════════════════════════

    def _send_cmd(self, cmd: str):
        """Send a raw command string to the H7."""
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log_warn("Cannot send to H7: serial port is not connected.")
            return
        try:
            self.ser.write((cmd + "\r\n").encode("utf-8"))
            self._log_tx(cmd)
        except Exception as e:
            self._log_err(f"Send failed: {e}")

    def _send_h7_input(self):
        """Send the text from the H7 input field to the serial port."""
        text = self._h7_input.text().strip()
        if not text:
            return
        self._h7_input.clear()
        if not self.connected:
            self._log_h7("[TX-H7]", f"{text}  (not sent - not connected)", "#C9831A")
            self._log_warn("Cannot send to H7: serial port is not connected.")
        else:
            self._send_cmd(text)
        self.setFocus()  # return focus to main window so WASD works again

    # ══════════════════════════════════════════════════════════════════════
    #  Mode / Value Management
    # ══════════════════════════════════════════════════════════════════════

    def _get_current_value(self) -> int:
        return self.current_rpm if self.mode == "RPM" else self.current_pwm

    def _set_mode(self, new_mode: str):
        if new_mode == self.mode:
            return
        self.mode = new_mode
        self._lbl_mode.setText(new_mode)
        if new_mode == "RPM":
            self._lbl_mode.setStyleSheet(
                "color: #D4AF37; font-size: 16px; font-weight: bold;"
            )
            self._lbl_value_label.setText("RPM Value:")
            self._lbl_value.setText(str(self.current_rpm))
            self._send_cmd("mode rpm")
        else:
            self._lbl_mode.setStyleSheet(
                "color: #FFD66B; font-size: 16px; font-weight: bold;"
            )
            self._lbl_value_label.setText("PWM Value:")
            self._lbl_value.setText(str(self.current_pwm))
            self._send_cmd("mode pwm")
        self._log_info(f"Mode changed to {new_mode}")
        self._lbl_qs_mode.setText(f"Mode: {new_mode}")

    def _toggle_mode(self):
        self._set_mode("PWM" if self.mode == "RPM" else "RPM")

    def _adjust_value(self, delta: int):
        if self.mode == "RPM":
            self.current_rpm = max(0, min(self.RPM_MAX, self.current_rpm + delta))
            self._lbl_value.setText(str(self.current_rpm))
            self._log_info(f"RPM value set to {self.current_rpm}")
        else:
            self.current_pwm = max(0, min(self.PWM_MAX, self.current_pwm + delta))
            self._lbl_value.setText(str(self.current_pwm))
            self._log_info(f"PWM value set to {self.current_pwm}")

    # ══════════════════════════════════════════════════════════════════════
    #  Movement Command Mapping
    # ══════════════════════════════════════════════════════════════════════

    def _movement_cmd(self, key: str) -> str:
        """Return the command string for a movement key in the current mode."""
        val = self._get_current_value()
        if self.mode == "RPM":
            return {"W": f"f{val}", "S": f"b{val}", "A": f"l{val}", "D": f"r{val}"}[key]
        else:
            return {"W": f"fd{val}", "S": f"bd{val}", "A": f"ld{val}", "D": f"rd{val}"}[key]

    # ══════════════════════════════════════════════════════════════════════
    #  Keyboard Handling
    # ══════════════════════════════════════════════════════════════════════

    def eventFilter(self, obj, event):
        """Event filter only for H7 input field."""
        if obj is self._h7_input:
            if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Escape:
                self.setFocus()
                return True
            return False  # let QLineEdit handle its own events
        return super().eventFilter(obj, event)

    MOVEMENT_KEYS = ("W", "S", "A", "D")

    def _key_to_id(self, event) -> str | None:
        """Convert a key event to a string identifier."""
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
        if text == "I":
            return "I"
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
            self._update_motion_indicator(self._active_move_key)
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "Space":
            self._send_cmd("stop")
            self._update_motion_indicator(None)
        elif key_id == "X":
            self._send_cmd("brake")
            self._update_motion_indicator(None)
        elif key_id == "M":
            self._toggle_mode()
        elif key_id == "I":
            self._send_cmd("identify")
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
            self._update_motion_indicator(self._active_move_key)
        elif key_id in ("Shift", "Ctrl"):
            self._active_modifier = None
            if not self._active_move_key:
                self._repeat_timer.stop()

        super().keyReleaseEvent(event)

    def _repeat_movement(self):
        """Called every 500 ms by the repeat timer."""
        if self._active_move_key:
            self._send_cmd(self._movement_cmd(self._active_move_key))
        if self._active_modifier == "Shift":
            self._adjust_value(self.VALUE_STEP)
        elif self._active_modifier == "Ctrl":
            self._adjust_value(-self.VALUE_STEP)
        if not self._active_move_key and not self._active_modifier:
            self._repeat_timer.stop()

    # ══════════════════════════════════════════════════════════════════════
    #  Help Popup
    # ══════════════════════════════════════════════════════════════════════

    def _show_help_popup(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Earendil GUI Help")
        dlg.setMinimumWidth(520)
        dlg.setStyleSheet("""
            QDialog {
                background-color: #17171C;
                color: #C0C0C0;
                font-size: 13px;
            }
            QLabel {
                color: #C0C0C0;
            }
            QPushButton {
                background-color: #2A2A31;
                border: 1px solid #D4AF37;
                border-radius: 6px;
                padding: 8px 24px;
                color: #D4AF37;
                font-weight: bold;
                min-height: 28px;
            }
            QPushButton:hover {
                background-color: #3A3320;
            }
        """)

        layout = QVBoxLayout(dlg)
        layout.setSpacing(12)

        title = QLabel("Earendil — Rover Control GUI Help")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #D4AF37;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color: #5F5A4A;")
        layout.addWidget(line)

        keys_html = (
            "<table style='font-size:13px; color:#C0C0C0;' cellspacing='8'>"
            "<tr><td style='color:#FFD66B;'><b>W</b></td><td>Forward</td>"
            "<td style='color:#FFD66B;'><b>Space</b></td><td>Stop</td></tr>"
            "<tr><td style='color:#FFD66B;'><b>S</b></td><td>Backward</td>"
            "<td style='color:#FFD66B;'><b>X</b></td><td>Brake</td></tr>"
            "<tr><td style='color:#FFD66B;'><b>A</b></td><td>Left</td>"
            "<td style='color:#FFD66B;'><b>M</b></td><td>Toggle RPM/PWM</td></tr>"
            "<tr><td style='color:#FFD66B;'><b>D</b></td><td>Right</td>"
            "<td style='color:#FFD66B;'><b>I</b></td><td>Identify</td></tr>"
            "<tr><td style='color:#FFD66B;'><b>LShift</b></td><td>Value +5</td>"
            "<td style='color:#FFD66B;'><b>LCtrl</b></td><td>Value -5</td></tr>"
            "</table>"
        )
        keys_label = QLabel(keys_html)
        keys_label.setTextFormat(Qt.RichText)
        layout.addWidget(keys_label)

        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet("color: #5F5A4A;")
        layout.addWidget(line2)

        mode_html = (
            "<table style='font-size:13px; color:#C0C0C0;' cellspacing='4'>"
            "<tr><td style='color:#D4AF37;'><b>RPM mode:</b></td>"
            "<td>W/S/A/D sends f/b/l/r&lt;number&gt;</td></tr>"
            "<tr><td style='color:#D4AF37;'><b>PWM mode:</b></td>"
            "<td>W/S/A/D sends fd/bd/ld/rd&lt;number&gt;</td></tr>"
            "</table>"
            "<br>"
            "<span style='color:#8E8E93;'>Held key repeats every 500 ms</span>"
        )
        mode_label = QLabel(mode_html)
        mode_label.setTextFormat(Qt.RichText)
        layout.addWidget(mode_label)

        line3 = QFrame()
        line3.setFrameShape(QFrame.HLine)
        line3.setStyleSheet("color: #5F5A4A;")
        layout.addWidget(line3)

        console_html = (
            "<table style='font-size:13px; color:#C0C0C0;' cellspacing='4'>"
            "<tr><td style='color:#D4AF37;'><b>H7 Console:</b></td>"
            "<td>Shows serial TX/RX with the STM32H723</td></tr>"
            "<tr><td style='color:#8E8E93;'><b>GUI Console:</b></td>"
            "<td>Shows GUI-local messages, warnings, and errors</td></tr>"
            "</table>"
        )
        console_label = QLabel(console_html)
        console_label.setTextFormat(Qt.RichText)
        layout.addWidget(console_label)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dlg.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignCenter)

        dlg.exec()

    # ══════════════════════════════════════════════════════════════════════
    #  Cleanup
    # ══════════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        self._repeat_timer.stop()
        self._stop_reader()
        self._close_serial()
        super().closeEvent(event)


# ════════════════════════════════════════════════════════════════════════════
#  Entry Point
# ════════════════════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")   # Fusion allows full stylesheet control
    window = EarendilControlGui()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
