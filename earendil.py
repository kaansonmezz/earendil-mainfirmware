#!/usr/bin/env python3
"""
Earendil - Rover Control GUI for STM32H723ZG Main Controller
============================================================
Single-file PySide6 + pyserial application.
Communicates with the H7 firmware over USART3 / ST-LINK VCP.

Controls:
    W/A/S/D   -> forward / left / backward / right
    Space     -> stop
    X         -> brake
    M         -> toggle mode (RPM / DUTY)

    I         -> identify
    LShift    -> increase RPM/DUTY by +5
    LCtrl     -> decrease RPM/DUTY by -5

Run:
    python earendil.py
"""

import re
import sys
import time
import threading
from collections import deque

# -- Dependency check -------------------------------------------------------
_missing = []
try:
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QPushButton, QComboBox, QLineEdit,
        QGroupBox, QTextEdit, QTableWidget, QTableWidgetItem,
        QDialog, QHeaderView, QSplitter, QFrame, QSizePolicy,
        QFormLayout, QSpinBox, QDoubleSpinBox, QGridLayout,
    )
    from PySide6.QtCore import Qt, QTimer, Signal, QObject, QThread, QEvent
    from PySide6.QtGui import (
        QFont, QColor, QTextCursor, QKeyEvent, QKeySequence,
        QPainter, QPixmap, QImage,
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


# ============================================================================
#  Background Logo Watermark
#  Paints a low-opacity centered logo behind the GUI content in paintEvent().
# ============================================================================

class LogoBackgroundWidget(QWidget):
    """Central widget that paints a transparent logo watermark.

    The logo is drawn in the bottom-left corner at low opacity; child widgets
    (buttons, tables, consoles) paint on top via Qt's normal compositing, so
    the logo always sits behind the UI content.
    """

    @staticmethod
    def _trim_alpha(pixmap: QPixmap) -> QPixmap:
        """Crop transparent margins so the visible artwork fills the pixmap.

        Without this, a logo PNG with transparent padding makes bottom-left
        corner positioning look wrong (the artwork sits in the middle of the
        pixmap area, not at the corner).
        """
        if pixmap.isNull():
            return pixmap
        img = pixmap.toImage().convertToFormat(QImage.Format_ARGB32_Premultiplied)
        w, h = img.width(), img.height()
        min_x, min_y = w, h
        max_x, max_y = -1, -1
        for y in range(h):
            for x in range(w):
                if img.pixel(x, y) != 0:  # non-fully-transparent pixel
                    if x < min_x: min_x = x
                    if x > max_x: max_x = x
                    if y < min_y: min_y = y
                    if y > max_y: max_y = y
        if max_x < 0:  # fully transparent image
            return pixmap
        return QPixmap.fromImage(img.copy(min_x, min_y,
                                          max_x - min_x + 1,
                                          max_y - min_y + 1))

    def __init__(self, logo_path: str, opacity: float = 0.06, parent=None):
        super().__init__(parent)
        self.logo = self._trim_alpha(QPixmap(logo_path))
        self.opacity = opacity
        # Background base color is theme-aware; updated by the main window's
        # _apply_theme().  Default is the dark theme base.
        self.background_color = "#101014"
        self._logo_missing = self.logo.isNull()
        if self._logo_missing:
            print(f"[GUI-WARN] Background logo could not be loaded: {logo_path}")

    def set_background_color(self, color: str):
        """Update the painted base color and trigger a repaint."""
        self.background_color = color
        self.update()

    def set_opacity(self, opacity: float):
        """Adjust the watermark opacity (e.g. lighter in light theme)."""
        self.opacity = opacity
        self.update()

    def paintEvent(self, event):
        # Paint a solid base so the watermark always sits on the theme
        # background, then draw the logo in the bottom-left corner at low
        # opacity. Child widgets composite over this in Qt's normal paint order.
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(self.background_color))

        if self.logo.isNull():
            return

        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        painter.setOpacity(self.opacity)

        target_width = int(self.width() * 0.12)
        if target_width <= 0:
            return
        scaled = self.logo.scaledToWidth(target_width, Qt.SmoothTransformation)

        # Bottom-left corner, flush against the edges.
        x = 0
        y = self.height() - scaled.height()

        painter.drawPixmap(x, y, scaled)


# ============================================================================
#  Serial Reader Thread
# ============================================================================

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


# ============================================================================
#  H7 UART Error Log Parsing
#  Matches firmware output from Core/Src/motor_uart_dma.c:
#    [ERROR] <UART> UART error code: 0x00000004
#    [ERROR] <UART> UART error still unresolved: 0x00000004
#    [ERROR] <UART> error: <CODE> - <Description>
#    [INFO]  <UART> RX recovered after UART error
# ============================================================================
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

# -- F411 motor telemetry patterns ------------------------------------------
#   New H7 format:  [TEL][FL] RPM:60,T:0,...
#   Legacy format:  [INFO] [USART2_RX] RPM:60,T:0,...
# Both may have a leading [INFO] prefix from the H7 logger.
_RE_MOTOR_TEL_TAGGED = re.compile(
    r"(?:\[INFO\]\s*)?\[TEL\]\[(FL|FR|RL|RR)\]\s+(RPM:.*)$"
)
_RE_MOTOR_TEL_UART = re.compile(
    r"(?:\[INFO\]\s*)?\[(USART2_RX|UART4_RX|UART5_RX|UART7_RX)\]\s+(RPM:.*)$"
)

# -- Operating mode confirmation from H7 firmware (command_handler.c) ------
#   The firmware logger (logger.c) prepends a level tag to every line, so
#   the actual serial output looks like:
#       [INFO] [MODE] DISARM active, motion commands locked
#       [INFO] [MODE] MANUAL active
#       [INFO] [MODE] AUTONOMOUS active
# This is the single source of truth for the GUI Operating Mode indicator.
_RE_OP_MODE_CONFIRM = re.compile(
    r"\[MODE\]\s+(DISARM|MANUAL|AUTONOMOUS)\s+active\b"
)

# -- IMU telemetry pattern ---------------------------------------------------
#   [INFO] MPU_IMU,AX:<mg>,AY:<mg>,AZ:<mg>,GX:<mdps>,GY:<mdps>,GZ:<mdps>,TC:<cx100>,...
def _parse_kv_payload(line: str, marker: str) -> dict[str, int] | None:
    """Generic key:value parser for firmware telemetry lines.

    Returns a dict of {key: int_value} for all ``KEY:VALUE`` pairs found
    after *marker* in *line*, or ``None`` if *marker* is not present.
    Unknown keys and non-integer values are silently skipped.
    """
    if marker not in line:
        return None
    payload = line.split(marker, 1)[1]
    result: dict[str, int] = {}
    for item in payload.split(","):
        if ":" not in item:
            continue
        key, value = item.split(":", 1)
        key = key.strip()
        value = value.strip()
        try:
            result[key] = int(value)
        except ValueError:
            continue
    return result if result else None


# ============================================================================
#  F411 Motor Tuning Settings Dialog
#  Placeholder / planning UI for configuring F411 motor parameters from the GUI.
#  Commands are built by the main window's centralized placeholder builder
#  (EarendilControlGui.build_f411_tuning_commands) and routed through the
#  existing _send_cmd() serial path, so logging / disconnected handling stay
#  consistent with the rest of the GUI.  Nothing here assumes the final H7/F411
#  raw motor forwarding protocol is complete - the format lives in one place:
#  build_f411_tuning_commands().
# ============================================================================

class MotorSettingsDialog(QDialog):
    """F411 Motor Tuning Settings dialog.

    Opened from the Motor State section's *Settings* button.  This is a
    placeholder / planning UI: the operator tweaks 8 slots of Base PWM /
    Boost PWM / Boost MS, Ramp Up/Down, Kp/Ki, Telemetry period, and an
    optional custom command, then clicks one of the per-motor send buttons.

    Commands are NOT hardcoded in callbacks: collect_f411_tuning_settings()
    reads the form, EarendilControlGui.build_f411_tuning_commands() builds
    the placeholder protocol lines, and send_f411_tuning_command() routes
    them through the existing _send_cmd() serial path.  Update those
    centralized functions when the real H7 raw motor forwarding protocol
    and F411 firmware parser are finalized.
    """

    NUM_SLOTS = 8
    MOTOR_TAGS = ("FL", "FR", "RL", "RR")

    def __init__(self, main_gui: "EarendilControlGui", parent=None):
        super().__init__(parent)
        self._gui = main_gui
        self.setWindowTitle("F411 Motor Tuning Settings")
        # Wider than the previous dialog so the 8-row tuning table reads well.
        self.setMinimumWidth(560)
        self._apply_theme_style()

        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(12, 12, 12, 12)

        # -- 8-row tuning table: # | Base PWM | Boost PWM -------------
        # Note: Boost MS is a single global field (in the form below), NOT
        # one per slot.  See the layout comment in _build_form_group().
        root.addWidget(self._build_tuning_table_group())

        # -- Global settings: Boost MS / Kick Duty / Kick MS / Ramp /
        #     PI / Telemetry -------------------------------------------
        root.addWidget(self._build_form_group())

        # -- Optional custom command ------------------------------------
        root.addWidget(self._build_custom_group())

        # -- Send buttons (per-motor direct send + All) ----------------
        root.addLayout(self._build_send_row())

        # -- Read buttons (placeholder) -------------------------------
        root.addLayout(self._build_read_row())

        # -- Reset / Close ----------------------------------------------
        root.addLayout(self._build_utility_row())

        # Default values used by Reset Fields and dialog initialisation.
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
            "custom":    "",
        }
        self._reset_fields(log=False)

    # ======================================================================
    #  UI builders
    # ======================================================================

    def _build_tuning_table_group(self) -> QGroupBox:
        """Horizontal tuning table: slots 1-8 left-to-right.

        Layout (3 rows x 9 columns):

            Slot:        1      2      3    ...    8
            Base PWM:  [  ]   [  ]   [  ]  ...  [  ]
            Boost PWM: [  ]   [  ]   [  ]  ...  [  ]

        Boost MS / Kick Duty / Kick MS are NOT here - they are single
        global fields in _build_form_group().
        """
        grp = QGroupBox("Tuning Slots")
        lay = QGridLayout(grp)
        lay.setSpacing(6)
        lay.setContentsMargins(10, 14, 10, 10)

        # Row 0 - "Slot:" label + slot numbers 1..8
        lay.addWidget(QLabel("Slot:"), 0, 0)
        for i in range(self.NUM_SLOTS):
            lbl = QLabel(str(i + 1))
            lbl.setAlignment(Qt.AlignCenter)
            lay.addWidget(lbl, 0, i + 1)

        # Row 1 - "Base PWM:" label + 8 input fields
        lay.addWidget(QLabel("Base PWM:"), 1, 0)
        self._base_edits = []
        for i in range(self.NUM_SLOTS):
            edit = QLineEdit()
            edit.setPlaceholderText("0")
            edit.setFixedWidth(60)
            edit.setAlignment(Qt.AlignCenter)
            lay.addWidget(edit, 1, i + 1)
            self._base_edits.append(edit)

        # Row 2 - "Boost PWM:" label + 8 input fields
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
        """Global tuning form fields.

        Single Boost MS field (shared across all slots), Kick Duty, Kick MS,
        Ramp Up / Ramp Down, Kp / Ki, Telemetry Period - all QLineEdit only
        (no spinbox arrows).  Values are raw text so the placeholder builder
        can later emit any final protocol format freely.
        """
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

    def _build_custom_group(self) -> QGroupBox:
        """Optional custom command text box."""
        grp = QGroupBox("Custom Command")
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(10, 14, 10, 10)
        self._custom_edit = QLineEdit()
        self._custom_edit.setPlaceholderText(
            "Free-text command line (appended after the motor tag)"
        )
        lay.addWidget(self._custom_edit, 1)
        return grp

    def _build_send_row(self) -> QHBoxLayout:
        """Per-motor direct send buttons + Send to All."""
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
        """Reset Fields + Close."""
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
        """Placeholder Read buttons - one per motor (no Read All)."""
        row = QHBoxLayout()
        row.setSpacing(6)
        for motor in self.MOTOR_TAGS:
            btn = QPushButton(f"Read {motor}")
            row.addWidget(btn, 1)
        return row

    # ======================================================================
    #  Theme styling
    # ======================================================================

    def _apply_theme_style(self):
        """Style the dialog using the active main-window palette so the popup
        stays visually consistent with the rest of the GUI (dark or light).
        Colors are pulled from self._gui._colors(); no hardcoded theme colors.
        """
        c = self._gui._colors()
        self.setStyleSheet(f"""
            QDialog {{
                background-color: {c['bg_main']};
                color: {c['text']};
                font-size: 13px;
                font-weight: {c['font_weight']};
            }}
            QLabel {{
                color: {c['text']};
            }}
            QGroupBox {{
                background-color: {c['bg_panel']};
                border: 1px solid {c['border']};
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 14px;
                color: {c['text']};
                font-weight: bold;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: {c['accent_gold']};
            }}
            QPushButton {{
                background-color: {c['bg_input']};
                border: 1px solid {c['accent_gold']};
                border-radius: 6px;
                padding: 6px 14px;
                color: {c['accent_gold']};
                font-weight: bold;
                min-height: 28px;
            }}
            QPushButton:hover {{
                background-color: {c['selection_bg']};
            }}
            QPushButton:pressed {{
                background-color: {c['pressed_bg']};
            }}
            QLineEdit {{
                background-color: {c['bg_input']};
                border: 1px solid {c['border']};
                border-radius: 4px;
                padding: 4px 8px;
                color: {c['text']};
                font-weight: {c['font_weight']};
            }}
        """)

    # ======================================================================
    #  Helpers
    # ======================================================================

    def collect_f411_tuning_settings(self) -> dict:
        """Read the entire dialog form into a plain dict.

        Returned dict shape (all values are raw strings typed by the operator
        - no up/down spinbox logic):
            {
                "base":      [str]*8,  # Base PWM 1..8
                "boost":     [str]*8,  # Boost PWM 1..8
                "boostms":   str,      # single global Boost MS field
                "kick_duty": str,
                "kick_ms":   str,
                "ramp_up":   str, "ramp_down": str,
                "kp":        str, "ki": str,
                "telper":    str,
                "custom":    str,
            }
        Keeping everything as strings leaves the placeholder builder free to
        emit any final protocol format (integers, floats, hex...) when the
        real H7/F411 parser is wired.  Boost MS is a single global value,
        not one per tuning slot.
        """
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
            "custom":    self._custom_edit.text().strip(),
        }

    def _resolve_motors(self, key: str) -> list:
        """Map a send-button key ("FL"/"FR"/"RL"/"RR"/"All") to the motor
        tag(s) used by the H7 terminal.  "All" now maps to the single
        tag "ALL" so the H7 firmware broadcasts once instead of four
        separate per-motor sends."""
        if key == "All":
            return ["ALL"]
        return [key] if key in self.MOTOR_TAGS else []

    def _reset_fields(self, log: bool = True):
        """Restore all text boxes to the default values."""
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
        self._custom_edit.clear()
        if log:
            self._gui._log_info("[F411-TUNE] Settings dialog fields reset to defaults")

    # ======================================================================
    #  Send handlers - single dispatch path for every send button
    # ======================================================================

    def _on_send_to(self, target: str):
        """Send the whole form (tuning slots + ramp + PI + telper + custom)
        to one target ("FL"/"FR"/"RL"/"RR"/"All") as a paced sequence.

        "All" generates ALL commands; individual targets generate per-motor
        commands.  All command building goes through the centralized
        build_f411_tuning_commands() helper.

        The sequence is: stop -> tuning commands -> spstat, sent one-by-one
        via QTimer pacing (TUNING_SEND_INTERVAL_MS) to avoid H7 terminal
        RX / motor TX FIFO overflow.  Send buttons are disabled during the
        sequence and re-enabled on completion.
        """
        motors = self._resolve_motors(target)
        if not motors:
            self._gui._log_warn(f"[F411-TUNE] Unknown send target {target!r}")
            return

        settings = self.collect_f411_tuning_settings()

        all_cmds: list[str] = []
        for motor in motors:
            # Stop before tuning so F411 accepts config changes.
            if motor == "ALL":
                all_cmds.append("stop")
            else:
                all_cmds.append(f"{motor} stop")

            # Tuning commands (base/boost/kickduty/kickms/ramp/pi/telper/custom).
            cmds = self._gui.build_f411_tuning_commands(motor, settings)
            all_cmds.extend(cmds)

            # spstat after tuning so the operator can verify F411 received.
            if motor == "ALL":
                all_cmds.extend(["FL spstat", "FR spstat", "RL spstat", "RR spstat"])
            else:
                all_cmds.append(f"{motor} spstat")

        self._gui.enqueue_f411_tuning_sequence(all_cmds, dialog=self)

    def _set_send_buttons_enabled(self, enabled: bool):
        """Enable/disable all tuning send buttons (called by the GUI
        during a paced tuning sequence)."""
        for btn in self._send_buttons.values():
            btn.setEnabled(enabled)
        if hasattr(self, "_btn_reset"):
            self._btn_reset.setEnabled(enabled)


# ============================================================================
#  IMU / MAG Settings Dialog
# ============================================================================

class ImuMagSettingsDialog(QDialog):
    """IMU / MAG Settings dialog.

    Contains all IMU/MAG command buttons organized in grouped sections.
    Opened from the main GUI's 'IMU Settings' button near the IMU table.
    """

    def __init__(self, main_gui: "EarendilControlGui", parent=None):
        super().__init__(parent)
        self._gui = main_gui
        self.setWindowTitle("IMU / MAG Settings")
        self.setMinimumWidth(420)
        self._apply_theme_style()

        root = QVBoxLayout(self)
        root.setSpacing(4)
        root.setContentsMargins(6, 6, 6, 6)

        # Grid layout for grouped sections
        grid = QGridLayout()
        grid.setSpacing(4)

        # Row 0: I2C/Detection | Init
        grid.addWidget(self._build_i2c_detection_group(), 0, 0)
        grid.addWidget(self._build_init_group(), 0, 1)

        # Row 1: MPU Read | MPU Bias
        grid.addWidget(self._build_mpu_read_group(), 1, 0)
        grid.addWidget(self._build_mpu_bias_group(), 1, 1)

        # Row 2: MPU Test | IMU Stream
        grid.addWidget(self._build_mpu_test_group(), 2, 0)
        grid.addWidget(self._build_imu_stream_group(), 2, 1)

        # Row 3: Telemetry Period | Gyro Filter
        grid.addWidget(self._build_telper_group(), 3, 0)
        grid.addWidget(self._build_gyro_filter_group(), 3, 1)

        # Row 4: Gyro Deadband | Gyro LPF
        grid.addWidget(self._build_gyro_deadband_group(), 4, 0)
        grid.addWidget(self._build_gyro_lpf_group(), 4, 1)

        # Row 5: MAG Read | Help
        grid.addWidget(self._build_mag_read_group(), 5, 0)
        grid.addWidget(self._build_help_group(), 5, 1)

        root.addLayout(grid)

        # Close button
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.close)
        root.addWidget(btn_close)

    # ==================================================================
    #  Helper: create a small command button
    # ==================================================================

    def _make_btn(self, label: str, cmd: str) -> QPushButton:
        """Create a compact button that sends a command when clicked."""
        btn = QPushButton(label)
        btn.setFixedHeight(24)
        btn.setStyleSheet("""
            QPushButton {
                padding: 2px 6px;
                font-size: 11px;
            }
        """)
        btn.clicked.connect(lambda checked=False, c=cmd: self._gui._send_cmd(c))
        return btn

    def _make_hbox(self, *widgets) -> QHBoxLayout:
        """Create a horizontal layout with tight spacing."""
        lay = QHBoxLayout()
        lay.setSpacing(3)
        lay.setContentsMargins(0, 0, 0, 0)
        for w in widgets:
            lay.addWidget(w)
        return lay

    # ==================================================================
    #  UI builders
    # ==================================================================

    def _build_i2c_detection_group(self) -> QGroupBox:
        grp = QGroupBox("I2C / Detection")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("Scan", "i2cscan"),
            self._make_btn("MPU WHO", "mpuwho"),
            self._make_btn("MAG WHO", "magwho"),
        ))
        return grp

    def _build_init_group(self) -> QGroupBox:
        grp = QGroupBox("Init")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        btn_all = QPushButton("Init All")
        btn_all.setFixedHeight(24)
        btn_all.setStyleSheet("QPushButton { padding: 2px 6px; font-size: 11px; }")
        btn_all.clicked.connect(self._init_all)
        lay.addLayout(self._make_hbox(
            self._make_btn("MPU", "mpuinit"),
            self._make_btn("MAG", "maginit"),
            btn_all,
        ))
        return grp

    def _build_mpu_read_group(self) -> QGroupBox:
        grp = QGroupBox("MPU Read")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("Raw", "mpuraw"),
            self._make_btn("Conv", "mpuconv"),
        ))
        return grp

    def _build_mpu_bias_group(self) -> QGroupBox:
        grp = QGroupBox("MPU Bias")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("Status", "mpubias"),
            self._make_btn("ON", "mpubiason"),
            self._make_btn("OFF", "mpubiasoff"),
            self._make_btn("Clear", "mpubiasclear"),
        ))
        return grp

    def _build_mpu_test_group(self) -> QGroupBox:
        grp = QGroupBox("MPU Test")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("Config", "mpucfgtest"),
            self._make_btn("Gyro", "mpugyrotest"),
        ))
        return grp

    def _build_imu_stream_group(self) -> QGroupBox:
        grp = QGroupBox("IMU Stream")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("ON", "imu stream on"),
            self._make_btn("OFF", "imu stream off"),
        ))
        self._lbl_stream_status = QLabel("Status: --")
        self._lbl_stream_status.setStyleSheet("color: #8E8E93; font-size: 10px;")
        lay.addWidget(self._lbl_stream_status)
        return grp

    def _build_telper_group(self) -> QGroupBox:
        grp = QGroupBox("Tel Period")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        row = QHBoxLayout()
        row.setSpacing(3)
        row.setContentsMargins(0, 0, 0, 0)
        self._spin_telper = QSpinBox()
        self._spin_telper.setRange(20, 5000)
        self._spin_telper.setValue(100)
        self._spin_telper.setFixedHeight(24)
        self._spin_telper.setFixedWidth(70)
        row.addWidget(self._spin_telper)
        row.addWidget(QLabel("ms"))
        btn = QPushButton("Set")
        btn.setFixedHeight(24)
        btn.setFixedWidth(40)
        btn.setStyleSheet("QPushButton { padding: 2px 6px; font-size: 11px; }")
        btn.clicked.connect(self._set_telper)
        row.addWidget(btn)
        row.addStretch()
        lay.addLayout(row)
        return grp

    def _build_gyro_filter_group(self) -> QGroupBox:
        grp = QGroupBox("Gyro Filter")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("Status", "imu gyrofilter status"),
            self._make_btn("ON", "imu gyrofilter on"),
            self._make_btn("OFF", "imu gyrofilter off"),
        ))
        self._lbl_gyro_filter_status = QLabel("Status: --")
        self._lbl_gyro_filter_status.setStyleSheet("color: #8E8E93; font-size: 10px;")
        lay.addWidget(self._lbl_gyro_filter_status)
        return grp

    def _build_gyro_deadband_group(self) -> QGroupBox:
        grp = QGroupBox("Deadband")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        row = QHBoxLayout()
        row.setSpacing(3)
        row.setContentsMargins(0, 0, 0, 0)
        self._spin_deadband = QSpinBox()
        self._spin_deadband.setRange(0, 2000)
        self._spin_deadband.setValue(250)
        self._spin_deadband.setFixedHeight(24)
        self._spin_deadband.setFixedWidth(70)
        row.addWidget(self._spin_deadband)
        row.addWidget(QLabel("mdps"))
        btn = QPushButton("Set")
        btn.setFixedHeight(24)
        btn.setFixedWidth(40)
        btn.setStyleSheet("QPushButton { padding: 2px 6px; font-size: 11px; }")
        btn.clicked.connect(self._set_deadband)
        row.addWidget(btn)
        row.addStretch()
        lay.addLayout(row)
        return grp

    def _build_gyro_lpf_group(self) -> QGroupBox:
        grp = QGroupBox("LPF")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        row = QHBoxLayout()
        row.setSpacing(3)
        row.setContentsMargins(0, 0, 0, 0)
        self._spin_lpf = QSpinBox()
        self._spin_lpf.setRange(1, 1000)
        self._spin_lpf.setValue(250)
        self._spin_lpf.setFixedHeight(24)
        self._spin_lpf.setFixedWidth(70)
        row.addWidget(self._spin_lpf)
        row.addWidget(QLabel("‰"))
        btn = QPushButton("Set")
        btn.setFixedHeight(24)
        btn.setFixedWidth(40)
        btn.setStyleSheet("QPushButton { padding: 2px 6px; font-size: 11px; }")
        btn.clicked.connect(self._set_lpf)
        row.addWidget(btn)
        row.addStretch()
        lay.addLayout(row)
        return grp

    def _build_mag_read_group(self) -> QGroupBox:
        grp = QGroupBox("MAG Read")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("Raw", "magraw"),
            self._make_btn("µT", "magimu"),
        ))
        return grp

    def _build_help_group(self) -> QGroupBox:
        grp = QGroupBox("Help")
        lay = QVBoxLayout(grp)
        lay.setSpacing(2)
        lay.setContentsMargins(6, 10, 6, 4)
        lay.addLayout(self._make_hbox(
            self._make_btn("IMU", "imu help"),
            self._make_btn("MAG", "maghelp"),
        ))
        return grp

    # ==================================================================
    #  Actions
    # ==================================================================

    def _init_all(self):
        """Send mpuinit then maginit sequentially."""
        self._gui._send_cmd("mpuinit")
        # Use QTimer to send second command after a short delay (non-blocking)
        QTimer.singleShot(200, lambda: self._gui._send_cmd("maginit"))

    def _set_telper(self):
        val = self._spin_telper.value()
        self._gui._send_cmd(f"imu telper {val}")

    def _set_deadband(self):
        val = self._spin_deadband.value()
        self._gui._send_cmd(f"imu deadband {val}")

    def _set_lpf(self):
        val = self._spin_lpf.value()
        self._gui._send_cmd(f"imu lpf {val}")

    # ==================================================================
    #  Theme
    # ==================================================================

    def _apply_theme_style(self):
        """Apply dark theme styling to the dialog."""
        self.setStyleSheet("""
            QDialog {
                background-color: #1e1e2e;
                color: #cdd6f4;
            }
            QGroupBox {
                background-color: #313244;
                border: 1px solid #45475a;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 10px;
                color: #cdd6f4;
                font-weight: bold;
                font-size: 11px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
                color: #89b4fa;
            }
            QPushButton {
                background-color: #45475a;
                border: 1px solid #585b70;
                border-radius: 3px;
                padding: 2px 6px;
                color: #cdd6f4;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #585b70;
            }
            QPushButton:pressed {
                background-color: #6c7086;
            }
            QSpinBox {
                background-color: #45475a;
                border: 1px solid #585b70;
                border-radius: 3px;
                padding: 2px;
                color: #cdd6f4;
                font-size: 11px;
            }
            QLabel {
                color: #cdd6f4;
                font-size: 11px;
            }
        """)

    # ==================================================================
    #  Status updates (called from main GUI when telemetry is parsed)
    # ==================================================================

    def update_stream_status(self, status: str):
        """Update the stream status label."""
        if hasattr(self, '_lbl_stream_status'):
            self._lbl_stream_status.setText(f"Stream: {status}")

    def update_gyro_filter_status(self, status: str):
        """Update the gyro filter status label."""
        if hasattr(self, '_lbl_gyro_filter_status'):
            self._lbl_gyro_filter_status.setText(f"Gyro Filter: {status}")

    def closeEvent(self, event):
        """Clean up when dialog is closed."""
        # Just accept the close, don't disconnect serial
        event.accept()


# ============================================================================
#  Main GUI
# ============================================================================

class EarendilControlGui(QMainWindow):
    """
    Main rover control window.
    Left side: control panels.  Right side: serial console.
    """

    # -- Theme palettes -------------------------------------------------------
    #  Semantic color keys used everywhere instead of raw hex.  The dark
    #  palette reproduces the original gold/dark theme exactly.  The light
    #  palette is a clean white/grey alternative that keeps the semantic
    #  accents (red/amber/green) meaningful.
    DARK_COLORS = {
        "bg_main":            "#101014",
        "bg_panel":           "transparent",
        "bg_input":           "#2A2A31",
        "bg_console":         "#0B0B0D",
        "bg_table":           "#0B0B0D",
        "table_header":       "#1E1E24",
        "gridline":            "#2A2A31",
        "text":               "#C0C0C0",
        "text_muted":         "#8E8E93",
        "border":             "#5F5A4A",
        "accent_gold":        "#D4AF37",
        "accent_gold_bright": "#FFD66B",
        "danger":             "#B00020",
        "danger_bright":      "#E02020",
        "success":            "#1e6e3e",
        "success_bright":     "#3CB371",
        "warning":            "#C9831A",
        "selection_bg":       "#3A3320",
        "selection_border":   "#8A6F2A",
        "pressed_bg":         "#4A4230",
        "led_inactive_bg":    "#2A2A31",
        "led_inactive_border":"#3A3A3A",
        "logo_opacity":       1.0,
        "manual_status_fg":   "#101014",
        "placeholder_text":   "#8E8E93",
        "font_weight":        "normal",
    }

    LIGHT_COLORS = {
        "bg_main":            "#F4F5F7",
        "bg_panel":           "transparent",
        "bg_input":           "#FFFFFF",
        "bg_console":         "#FAFAFA",
        "bg_table":           "#FFFFFF",
        "table_header":       "#E9EAEE",
        "gridline":            "#E0E0E0",
        "text":               "#000000",
        "text_muted":         "#000000",
        "border":             "#C7C9D1",
        "accent_gold":        "#000000",   # light theme: all text black
        "accent_gold_bright": "#000000",
        "danger":             "#C5221F",
        "danger_bright":      "#D93025",
        "success":            "#1E8E3E",
        "success_bright":     "#1E8E3E",
        "warning":            "#B06000",
        "selection_bg":       "#E8E2C9",
        "selection_border":   "#B8860B",
        "pressed_bg":         "#DADCE0",
        "led_inactive_bg":    "#DADCE0",
        "led_inactive_border":"#BDC1C6",
        "logo_opacity":       0.10,
        "manual_status_fg":   "#000000",
        "placeholder_text":   "#5F6368",
        "font_weight":        "bold",
    }

    THEMES = {"dark": DARK_COLORS, "light": LIGHT_COLORS}

    def _colors(self) -> dict:
        """Return the palette dict for the current theme."""
        return self.THEMES[self.current_theme]

    def _build_app_stylesheet(self) -> str:
        """Generate the project-wide QSS from the active theme palette.

        Reproduces the original dark theme exactly when the dark palette is
        active; produces the light theme otherwise.  Inline widget styles that
        need theme-aware values live in dedicated helper methods.
        """
        c = self._colors()
        return f"""
        QMainWindow {{
            background-color: {c['bg_main']};
        }}
        QWidget {{
            color: {c['text']};
            font-size: 13px;
            font-weight: {c['font_weight']};
        }}
        /* Container widgets stay transparent so the centered background logo
           (painted on the central widget) shows through the panel gaps. */
        QSplitter {{
            background: transparent;
        }}
        QWidget#sidePanel {{
            background: transparent;
        }}
        QGroupBox {{
            background-color: {c['bg_panel']};
            border: 1px solid {c['border']};
            border-radius: 6px;
            margin-top: 10px;
            margin-bottom: 0px;
            padding-top: 14px;
            padding-bottom: 0px;
            font-weight: bold;
            color: {c['text']};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 6px;
            color: {c['accent_gold']};
        }}
        QPushButton {{
            background-color: {c['bg_input']};
            border: 1px solid {c['border']};
            border-radius: 6px;
            padding: 6px 14px;
            min-height: 28px;
            color: {c['text']};
            font-weight: {c['font_weight']};
        }}
        QPushButton:hover {{
            background-color: {c['selection_bg']};
            border-color: {c['selection_border']};
        }}
        QPushButton:pressed {{
            background-color: {c['pressed_bg']};
        }}
        QComboBox, QLineEdit {{
            background-color: {c['bg_input']};
            border: 1px solid {c['border']};
            border-radius: 4px;
            padding: 4px 8px;
            color: {c['text']};
            font-weight: {c['font_weight']};
        }}
        QComboBox QAbstractItemView {{
            background-color: {c['bg_input']};
            border: 1px solid {c['border']};
            selection-background-color: {c['selection_bg']};
            color: {c['text']};
        }}
        QTableWidget {{
            background-color: {c['bg_table']};
            border: 1px solid {c['border']};
            border-radius: 4px;
            gridline-color: {c['gridline']};
            selection-background-color: {c['selection_bg']};
            color: {c['text']};
            font-weight: {c['font_weight']};
        }}
        QTableWidget::item {{
            padding: 4px;
        }}
        QHeaderView::section {{
            background-color: {c['table_header']};
            color: {c['accent_gold']};
            border: none;
            border-right: 1px solid {c['border']};
            border-bottom: 1px solid {c['border']};
            padding: 4px;
            font-weight: bold;
        }}
        QTextEdit {{
            background-color: {c['bg_console']};
            border: 1px solid {c['border']};
            border-radius: 4px;
            color: {c['text']};
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px;
            font-weight: {c['font_weight']};
        }}
        QSplitter::handle {{
            background-color: {c['border']};
            width: 3px;
        }}
        """

    # -- Constants ----------------------------------------------------------
    REPEAT_INTERVAL_MS = 500
    TUNING_SEND_INTERVAL_MS = 100
    # How long the GUI waits for an H7 confirmation of an operating-mode
    # command before warning that the mode change was not confirmed.
    OP_MODE_CONFIRM_TIMEOUT_MS = 3000
    DEFAULT_RPM_FB = 30
    DEFAULT_RPM_ROT = 100
    DEFAULT_PWM_FB = 1000
    DEFAULT_PWM_ROT = 2500
    RPM_MAX = 200
    PWM_MAX = 4000
    VALUE_STEP = 5
    DUTY_STEP = 100

    # -- Motor table row index ---------------------------------------------
    MOTOR_ROW = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}

    # -- UART -> motor mapping (must match H7 firmware) ---------------------
    #   app_config.h        : huart2=FL, huart4=FR, huart7=RL, huart5=RR
    #   motor_uart_dma.c    : USART2=FL, UART4=FR,  UART7=RL,  UART5=RR
    #   motor_tx_dma.c      : same mapping
    UART_TO_MOTOR = {
        "USART2": "FL",
        "UART4":  "FR",
        "UART7":  "RL",
        "UART5":  "RR",
    }

    # Recognized UART error code prefixes (HAL UART error flags)
    UART_ERROR_CODES = {"FE", "NE", "ORE", "PE", "DMA", "RTO"}

    # -- Motor table column indices -----------------------------------------
    MOTOR_COL = {
        "motor": 0,
        "current_rpm": 1,
        "target_rpm": 2,
        "drive_duty": 3,
        "direction": 4,
        "motor_state": 5,
        "control_mode": 6,
        "brake_status": 7,
        "fault_code": 8,
        "hall_sensor": 9,
        "target_pwm": 10,
        "applied_pwm": 11,
        "dropped_commands": 12,
        "received_uart_bytes": 13,
        "error": 14,
        "link": 15,
    }
    MOTOR_COL_HEADERS = [
        "Motor", "RPM", "Target RPM", "Duty",
        "Dir", "State", "Control Mode", "Brake",
        "Fault", "Hall", "Target PWM", "Applied PWM",
        "Dropped Commands", "RXB", "Error", "Link",
    ]

    # UART RX suffix -> motor tag (for legacy [USART2_RX] format)
    UART_RX_TO_MOTOR = {
        "USART2_RX": "FL",
        "UART4_RX":  "FR",
        "UART7_RX":  "RL",
        "UART5_RX":  "RR",
    }

    # F411 telemetry display translations
    _APP_PH_MAP = {"0": "Stopped", "1": "Running", "2": "Brake", "3": "Idle", "4": "Error"}
    _DIR_MAP = {"F": "Forward", "R": "Reverse", "N": "Neutral / No Direction"}
    _SP_MAP = {"0": "Duty/PWM Mode", "1": "RPM Control Mode"}
    _BRAKE_MAP = {"0": "Brake Off", "1": "Brake Active"}

    # -- Operating mode (DISARM / MANUAL / AUTONOMOUS) ---------------------
    #   drive/control mode (RPM/DUTY) is a separate concept handled by _set_mode.
    #   Commands are sent over the same H7 terminal serial path as other cmds.
    OPERATING_MODES = {
        "disarm": {
            "label": "DISARM",
            "command": "mode disarm",
            "color": "red",
            "status_bg": "#B00020",
            "status_fg": "#FFFFFF",
            "led": "#E02020",
        },
        "manual": {
            "label": "MANUAL",
            "command": "mode manual",
            "color": "yellow",
            "status_bg": "#FFD66B",
            "status_fg": "#101014",
            "led": "#FFD66B",
        },
        "auto": {
            "label": "AUTONOMOUS",
            "command": "mode auto",
            "color": "green",
            "status_bg": "#1E8E3E",
            "status_fg": "#FFFFFF",
            "led": "#3CB371",
        },
    }
    # Order of LEDs left->right: red, yellow, green
    OPERATING_MODE_LED_KEYS = ("disarm", "manual", "auto")

    # Movement key priority: most-recently-pressed wins
    MOVE_KEYS = {
        Qt.Key_W: "W",
        Qt.Key_S: "S",
        Qt.Key_A: "A",
        Qt.Key_D: "D",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Earendil - Rover Control")
        self.setMinimumSize(1100, 650)

        # -- State ----------------------------------------------------------
        self.current_theme = "dark"            # "dark" or "light"

        self.ser: serial.Serial | None = None
        self.reader_thread: SerialReaderThread | None = None
        self.connected = False

        self.mode = "RPM"               # "RPM" or "DUTY"
        self.fb_rpm = self.DEFAULT_RPM_FB
        self.rot_rpm = self.DEFAULT_RPM_ROT
        self.fb_pwm = self.DEFAULT_PWM_FB
        self.rot_pwm = self.DEFAULT_PWM_ROT

        self._operating_mode = "disarm"          # confirmed mode (H7 is source of truth)
        self._pending_mode: str | None = None   # mode requested by user, awaiting H7 confirm

        self._active_move_key: str | None = None   # current movement key (W/A/S/D)
        self._move_held: set[str] = set()          # held movement keys
        self._move_order: deque[str] = deque()     # movement key press order
        self._active_modifier: str | None = None   # "Shift" or "Ctrl" if held
        self._keys_held: set[str] = set()          # ALL held keys (prevents duplicates)

        # -- Motor UART error tracking -------------------------------------
        # motor -> current UART error text (empty string = no active UART error)
        self._motor_uart_error_text: dict[str, str] = {"FL": "", "FR": "", "RL": "", "RR": ""}
        # uart -> decoded error parts accumulated within one report cycle
        self._uart_report_decoded: dict[str, list[str]] = {}
        # motor -> F411 fault code string ("0" = no fault)
        self._motor_fault_code: dict[str, str] = {"FL": "0", "FR": "0", "RL": "0", "RR": "0"}
        # motor -> last telemetry values dict (key -> display string)
        self._motor_telemetry: dict[str, dict[str, str]] = {
            m: {} for m in ("FL", "FR", "RL", "RR")
        }

        # -- F411 tuning paced-send state --------------------------------
        self._tuning_send_queue: deque[str] = deque()
        self._tuning_dialog_ref = None
        self._tuning_send_timer = QTimer(self)
        self._tuning_send_timer.setInterval(self.TUNING_SEND_INTERVAL_MS)
        self._tuning_send_timer.timeout.connect(
            self._send_next_f411_tuning_command)

        # -- IMU/MAG settings dialog reference ---------------------------
        self._imu_settings_dialog: ImuMagSettingsDialog | None = None

        # -- Build UI -------------------------------------------------------
        self._central = LogoBackgroundWidget("earendil_logo.png", opacity=1.0)
        central = self._central
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel
        left_panel = QWidget()
        left_panel.setObjectName("sidePanel")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        # -- Top row: Serial Connection (left) + Rover Status (right) ---
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(8)
        top_row.addWidget(self._build_connection_group())
        top_row.addWidget(self._build_rover_status_group(), 1)

        left_layout.addLayout(top_row)

        # -- Mode / Value  (left)  +  Operating Mode (right)  in one row --
        mode_op_row = QHBoxLayout()
        mode_op_row.setContentsMargins(0, 0, 0, 0)
        mode_op_row.setSpacing(8)
        mode_op_row.addWidget(self._build_mode_value_group(), 1)
        mode_op_row.addWidget(self._build_operating_mode_group())
        left_layout.addLayout(mode_op_row)

        left_layout.addWidget(self._build_motor_table_group())

        imu_env_row = QHBoxLayout()
        imu_env_row.setContentsMargins(0, 0, 0, 0)
        imu_env_row.setSpacing(8)
        imu_env_row.addWidget(self._build_imu_group())
        imu_env_row.addWidget(self._build_env_sensor_group())
        imu_env_row.addStretch()
        left_layout.addLayout(imu_env_row)

        left_layout.addStretch()

        splitter.addWidget(left_panel)

        # Right panel - console
        splitter.addWidget(self._build_console_group())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # -- Repeat timer ---------------------------------------------------
        self._repeat_timer = QTimer(self)
        self._repeat_timer.setInterval(self.REPEAT_INTERVAL_MS)
        self._repeat_timer.timeout.connect(self._repeat_movement)

        # -- Operating-mode confirmation timeout ---------------------------
        # Started when a mode button sends a command; if the H7 does not
        # reply with a `[MODE] ... active` line in time, we warn and keep
        # the previously confirmed mode (no optimistic UI change).
        self._pending_mode_timer = QTimer(self)
        self._pending_mode_timer.setSingleShot(True)
        self._pending_mode_timer.setInterval(self.OP_MODE_CONFIRM_TIMEOUT_MS)
        self._pending_mode_timer.timeout.connect(self._on_pending_mode_timeout)

        # Prevent buttons from stealing keyboard focus (Space must always reach keyPressEvent)
        for btn in self.findChildren(QPushButton):
            btn.setFocusPolicy(Qt.NoFocus)

        # H7 input handles its own keyboard events; main window handles the rest
        self._h7_input.installEventFilter(self)
        self.setFocusPolicy(Qt.StrongFocus)

        # Apply the active theme to all widgets (stylesheet + inline styles +
        # background logo).  Done last so all widgets exist before re-styling.
        self._apply_theme()

        self._log_info("Ready. Connect to rover to begin.")

    # ======================================================================
    #  UI Builders
    # ======================================================================

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
        # Initial dark-theme default; _apply_theme() re-styles for the active theme.
        self._btn_connect.setStyleSheet("QPushButton { background-color: #1e6e3e; color: #C0C0C0; }")
        self._btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self._btn_connect)

        self._lbl_status = QLabel("* Disconnected")
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

        c = self._colors()

        def _badge(initial: str, color: str) -> QLabel:
            lbl = QLabel(initial)
            lbl.setStyleSheet(self._style_badge(color))
            return lbl

        self._lbl_qs_mode = _badge("Mode: RPM", c['accent_gold'])
        lay.addWidget(self._lbl_qs_mode)

        self._lbl_qs_motion = _badge("Motion: IDLE", c['text_muted'])
        lay.addWidget(self._lbl_qs_motion)

        self._lbl_qs_port = _badge("Port: Disconnected", c['danger'])
        lay.addWidget(self._lbl_qs_port)

        lay.addStretch()

        # Theme toggle button - only changes visual theme, never sends a
        # serial command.  Text reflects the theme we will switch TO.
        self._btn_theme = QPushButton("Light Mode")
        self._btn_theme.setFixedWidth(100)
        self._btn_theme.clicked.connect(self._toggle_theme)
        lay.addWidget(self._btn_theme)
        return grp

    def _build_mode_value_group(self) -> QGroupBox:
        grp = QGroupBox("Mode / Value")
        grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay = QHBoxLayout(grp)

        lay.addWidget(QLabel("Mode:"))
        self._lbl_mode = QLabel("RPM")
        self._lbl_mode.setStyleSheet(
            "color: #D4AF37; font-size: 16px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_mode)

        lay.addSpacing(20)
        self._lbl_fb_label = QLabel("FB RPM:")
        lay.addWidget(self._lbl_fb_label)
        self._lbl_fb_value = QLabel(str(self.fb_rpm))
        self._lbl_fb_value.setStyleSheet(
            "color: #FFD66B; font-size: 18px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_fb_value)

        lay.addSpacing(10)
        self._lbl_rot_label = QLabel("ROT RPM:")
        lay.addWidget(self._lbl_rot_label)
        self._lbl_rot_value = QLabel(str(self.rot_rpm))
        self._lbl_rot_value.setStyleSheet(
            "color: #FFD66B; font-size: 18px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_rot_value)

        lay.addSpacing(10)
        lay.addWidget(QLabel("Shift/Ctrl:FB +/- | Num+/Num-:ROT +/-"))

        lay.addStretch()

        btn_rpm = QPushButton("Mode RPM")
        btn_rpm.clicked.connect(lambda: self._set_mode("RPM"))
        lay.addWidget(btn_rpm)

        btn_pwm = QPushButton("Mode DUTY")
        btn_pwm.clicked.connect(lambda: self._set_mode("DUTY"))
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

    def _build_operating_mode_group(self) -> QGroupBox:
        grp = QGroupBox("Operating Mode")
        grp.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        grp.setMaximumWidth(420)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(8)

        # -- Left: three LEDs (red / yellow / green) ----------------------
        leds_col = QVBoxLayout()
        leds_col.setContentsMargins(0, 0, 0, 0)
        leds_col.setSpacing(5)
        self._led_red = self._make_led()
        self._led_yellow = self._make_led()
        self._led_green = self._make_led()
        leds_col.addWidget(self._led_red)
        leds_col.addWidget(self._led_yellow)
        leds_col.addWidget(self._led_green)
        leds_col.addStretch()
        lay.addLayout(leds_col)

        # -- Right: status box on top, three buttons below ----------------
        right_col = QVBoxLayout()
        right_col.setContentsMargins(0, 0, 0, 0)
        right_col.setSpacing(5)

        self._lbl_op_mode_status = QLabel("DISARM")
        self._lbl_op_mode_status.setAlignment(Qt.AlignCenter)
        self._lbl_op_mode_status.setFixedHeight(34)
        self._lbl_op_mode_status.setFixedWidth(380)
        # Initial styling follows the confirmed operating mode; _apply_theme()
        # and _update_operating_mode_ui() keep it theme-aware afterwards.
        self._style_operating_mode_status(self.OPERATING_MODES["disarm"])
        right_col.addWidget(self._lbl_op_mode_status)

        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.setSpacing(5)

        btn_disarm = QPushButton("DISARM")
        btn_disarm.clicked.connect(lambda: self._set_operating_mode("disarm"))
        btn_row.addWidget(btn_disarm, 1)

        btn_manual = QPushButton("MANUAL")
        btn_manual.clicked.connect(lambda: self._set_operating_mode("manual"))
        btn_row.addWidget(btn_manual, 1)

        btn_auto = QPushButton("AUTONOMOUS")
        btn_auto.clicked.connect(lambda: self._set_operating_mode("auto"))
        btn_row.addWidget(btn_auto, 1)

        right_col.addLayout(btn_row)
        lay.addLayout(right_col, 1)

        # Initialize indicators to the default operating mode.
        self._update_operating_mode_ui(self._operating_mode)
        return grp

    def _build_motor_table_group(self) -> QGroupBox:
        grp = QGroupBox("Motor State")
        grp.setStyleSheet("""
            QGroupBox {
                padding-bottom: 0px;
                margin-bottom: 0px;
            }
        """)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 0)
        lay.setSpacing(0)

        num_cols = len(self.MOTOR_COL_HEADERS)
        self._motor_table = QTableWidget(4, num_cols)
        self._motor_table.setHorizontalHeaderLabels(self.MOTOR_COL_HEADERS)
        headers = self._motor_table.horizontalHeader()
        if headers:
            headers.setSectionResizeMode(QHeaderView.Stretch)

        motors = ["FL", "FR", "RL", "RR"]
        for row, name in enumerate(motors):
            self._motor_table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, num_cols):
                self._motor_table.setItem(row, col, QTableWidgetItem("--"))

        self._motor_table.setVerticalHeaderLabels([])
        self._motor_table.verticalHeader().setVisible(False)
        self._motor_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._motor_table.setFocusPolicy(Qt.NoFocus)
        self._motor_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._motor_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._motor_table.setMaximumHeight(150)
        self._motor_table.setMinimumHeight(150)
        lay.addWidget(self._motor_table)

        # -- Settings button (opens F411 Motor Tuning Settings dialog) -----
        # Right-aligned so it reads as "table -> settings" within the group.
        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.setSpacing(0)
        btn_row.addStretch()

        self._btn_motor_settings = QPushButton("Settings")
        self._btn_motor_settings.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        self._btn_motor_settings.clicked.connect(self._open_motor_settings)
        btn_row.addWidget(self._btn_motor_settings)
        lay.addLayout(btn_row)
        return grp

    # -- 9-axis IMU placeholder --------------------------------------------
    #   3 accel (X/Y/Z) + 3 gyro (X/Y/Z) + 3 mag (X/Y/Z).
    #   Values shown as "--" until firmware sends real IMU data; the parser
    #   hook (_parse_imu_line) is wired and updates the converted table.
    IMU_FIELDS = ("AX", "AY", "AZ", "GX", "GY", "GZ", "MX", "MY", "MZ")

    # Row indices for the IMU table
    _IMU_ROW = {"Accel": 0, "Gyro": 1, "Mag": 2, "Temp": 3}

    def _build_imu_group(self) -> QGroupBox:
        grp = QGroupBox("IMU")
        grp.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        grp.setStyleSheet("""
            QGroupBox {
                padding-bottom: 0px;
                margin-bottom: 0px;
            }
        """)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 0)
        lay.setSpacing(0)

        self._imu_table = QTableWidget(4, 4)
        self._imu_table.setHorizontalHeaderLabels(["Sensor", "X", "Y", "Z"])
        headers = self._imu_table.horizontalHeader()
        if headers:
            headers.setSectionResizeMode(0, QHeaderView.Fixed)
            headers.setSectionResizeMode(1, QHeaderView.Fixed)
            headers.setSectionResizeMode(2, QHeaderView.Fixed)
            headers.setSectionResizeMode(3, QHeaderView.Fixed)
            headers.resizeSection(0, 65)
            headers.resizeSection(1, 65)
            headers.resizeSection(2, 65)
            headers.resizeSection(3, 65)
            headers.setFixedHeight(20)
            headers.setSectionsMovable(False)
            headers.setSortIndicatorShown(False)
        self._imu_table.verticalHeader().setVisible(False)
        self._imu_table.verticalHeader().setDefaultSectionSize(22)
        self._imu_table.verticalHeader().setMinimumWidth(0)
        self._imu_table.verticalHeader().setMaximumWidth(0)
        self._imu_table.setMinimumWidth(0)
        self._imu_table.setMaximumWidth(260)
        self._imu_table.setMaximumHeight(160)
        self._imu_table.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        labels = ["Accel", "Gyro", "Mag", "Temp"]
        for row, name in enumerate(labels):
            self._imu_table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 4):
                self._imu_table.setItem(row, col, QTableWidgetItem("--"))

        self._imu_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._imu_table.setFocusPolicy(Qt.NoFocus)
        self._imu_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._imu_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._imu_table.verticalHeader().hide()
        self._imu_table.horizontalHeader().setStretchLastSection(False)
        self._imu_table.setStyleSheet("""
            QTableWidget {
                padding: 0px;
                margin: 0px;
                border: 1px solid palette(mid);
                font-size: 12px;
            }
            QTableWidget::item {
                padding: 3px 5px;
            }
            QHeaderView::section {
                padding: 3px 5px;
                font-size: 11px;
            }
        """)
        lay.addWidget(self._imu_table)

        # IMU Settings button
        btn_imu_settings = QPushButton("Settings")
        btn_imu_settings.setFixedHeight(22)
        btn_imu_settings.setFixedWidth(90)
        btn_imu_settings.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        btn_imu_settings.clicked.connect(self._open_imu_settings)
        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.setSpacing(0)
        btn_row.addWidget(btn_imu_settings)
        btn_row.addStretch()
        lay.addLayout(btn_row)

        return grp

    # -- ENV Sensors placeholder -----------------------------------------------
    _ENV_ROW = {
        "Gas": 0, "Current": 1, "Temp": 2,
        "Humidity": 3, "UV": 4, "Pressure": 5,
    }
    _ENV_LABELS = ["Gas", "Current", "Temp", "Humidity", "UV", "Pressure"]
    _ENV_UNITS = ["Relative", "A", "°C", "%RH", "UV Index", "hPa"]

    def _build_env_sensor_group(self) -> QGroupBox:
        grp = QGroupBox("ENV Sensors")
        grp.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        grp.setStyleSheet("""
            QGroupBox {
                padding-bottom: 0px;
                margin-bottom: 0px;
            }
        """)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 0)
        lay.setSpacing(0)

        num_rows = len(self._ENV_LABELS)
        self._env_table = QTableWidget(num_rows, 4)
        self._env_table.setHorizontalHeaderLabels(["Sensor", "Value", "Unit", "Status"])
        headers = self._env_table.horizontalHeader()
        if headers:
            headers.setSectionResizeMode(0, QHeaderView.Fixed)
            headers.setSectionResizeMode(1, QHeaderView.Fixed)
            headers.setSectionResizeMode(2, QHeaderView.Fixed)
            headers.setSectionResizeMode(3, QHeaderView.Fixed)
            headers.resizeSection(0, 70)
            headers.resizeSection(1, 60)
            headers.resizeSection(2, 70)
            headers.resizeSection(3, 60)
            headers.setFixedHeight(20)
            headers.setSectionsMovable(False)
            headers.setSortIndicatorShown(False)
        self._env_table.verticalHeader().setVisible(False)
        self._env_table.verticalHeader().setDefaultSectionSize(22)
        self._env_table.verticalHeader().setMinimumWidth(0)
        self._env_table.verticalHeader().setMaximumWidth(0)
        self._env_table.setMinimumWidth(0)
        self._env_table.setMaximumWidth(260)
        self._env_table.setMaximumHeight(175)
        self._env_table.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        for row in range(num_rows):
            self._env_table.setItem(row, 0, QTableWidgetItem(self._ENV_LABELS[row]))
            self._env_table.setItem(row, 1, QTableWidgetItem("--"))
            self._env_table.setItem(row, 2, QTableWidgetItem(self._ENV_UNITS[row]))
            self._env_table.setItem(row, 3, QTableWidgetItem("N/A"))

        self._env_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._env_table.setFocusPolicy(Qt.NoFocus)
        self._env_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._env_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._env_table.verticalHeader().hide()
        self._env_table.horizontalHeader().setStretchLastSection(False)
        self._env_table.setStyleSheet("""
            QTableWidget {
                padding: 0px;
                margin: 0px;
                border: 1px solid palette(mid);
                font-size: 12px;
            }
            QTableWidget::item {
                padding: 3px 5px;
            }
            QHeaderView::section {
                padding: 3px 5px;
                font-size: 11px;
            }
        """)
        lay.addWidget(self._env_table)
        return grp

    def _set_env_cell(self, row: int, col: int, text: str):
        item = self._env_table.item(row, col)
        if item is not None:
            item.setText(text)

    def _set_imu_cell(self, row: int, col: int, text: str):
        item = self._imu_table.item(row, col)
        if item is not None:
            item.setText(text)

    def _update_motion_indicator(self, direction: str | None):
        """Update the Motion badge in Rover Status.  direction is one of W/S/A/D or None for IDLE."""
        mapping = {"W": "FORWARD", "S": "BACKWARD", "A": "LEFT", "D": "RIGHT"}
        text = mapping.get(direction, "IDLE")
        c = self._colors()
        color = c['accent_gold_bright'] if text != "IDLE" else c['text_muted']
        self._lbl_qs_motion.setText(f"Motion: {text}")
        self._lbl_qs_motion.setStyleSheet(self._style_badge(color))

    def _build_console_group(self) -> QGroupBox:
        grp = QGroupBox("Console")
        lay = QVBoxLayout(grp)

        console_splitter = QSplitter(Qt.Vertical)

        # -- H7 Console ------------------------------------------------
        h7_widget = QWidget()
        h7_lay = QVBoxLayout(h7_widget)
        h7_lay.setContentsMargins(0, 0, 0, 0)
        h7_lay.setSpacing(4)

        self._lbl_h7_console_title = QLabel("H7 Console")
        self._lbl_h7_console_title.setStyleSheet(
            "color: #D4AF37; font-weight: bold; font-size: 13px;"
        )
        h7_lay.addWidget(self._lbl_h7_console_title)

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

        # -- GUI Console -----------------------------------------------
        gui_widget = QWidget()
        gui_lay = QVBoxLayout(gui_widget)
        gui_lay.setContentsMargins(0, 0, 0, 0)
        gui_lay.setSpacing(4)

        self._lbl_gui_console_title = QLabel("GUI Console")
        self._lbl_gui_console_title.setStyleSheet(
            "color: #8E8E93; font-weight: bold; font-size: 13px;"
        )
        gui_lay.addWidget(self._lbl_gui_console_title)

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

    # ======================================================================
    #  Theme Management
    # ======================================================================
    #
    #  Theme switching is purely visual.  _toggle_theme() flips self.current_theme,
    #  regenerates the stylesheet, restyles every theme-aware inline style and
    #  repaints dynamic widgets to match the current state - WITHOUT touching
    #  runtime state (serial connection, values, operating mode, pending mode,
    #  console contents, motor/IMU tables).

    def _toggle_theme(self):
        """Switch between dark and light theme.  No serial I/O."""
        self.current_theme = "light" if self.current_theme == "dark" else "dark"
        self._apply_theme()
        self._btn_theme.setText("Light Mode" if self.current_theme == "dark" else "Dark Mode")
        label = "LIGHT" if self.current_theme == "light" else "DARK"
        self._log_info(f"Theme switched to {label}")

    def _apply_theme(self):
        """Apply the active palette to the global stylesheet, theme-aware
        inline widget styles, the background logo, and re-render dynamic
        widgets according to the current runtime state.  Runtime state is
        never changed here.
        """
        self.setStyleSheet(self._build_app_stylesheet())

        c = self._colors()

        # Background logo base color + opacity for the active theme.
        self._central.set_background_color(c['bg_main'])
        self._central.set_opacity(c['logo_opacity'])

        # -- Static (builder-set) widgets re-styled to the active theme --
        self._style_connection_button()
        self._style_connection_status()
        self._style_console_widgets()
        self._style_help_button()
        if hasattr(self, "_btn_motor_settings"):
            self._style_motor_settings_button()
        if hasattr(self, "_lbl_op_mode_status"):
            self._update_operating_mode_ui(self._operating_mode)

        # -- Dynamic state-driven widgets re-rendered to the active theme --
        # Quick-status mode badge (RPM/DUTY)
        if hasattr(self, "_lbl_qs_mode"):
            self._lbl_qs_mode.setText(f"Mode: {self.mode}")
            self._lbl_qs_mode.setStyleSheet(self._style_badge(c['accent_gold']))
        # Quick-status motion badge
        if hasattr(self, "_lbl_qs_motion"):
            self._update_motion_indicator(self._active_move_key)
        # Quick-status port badge + connection status label
        if self.connected and self.ser and self.ser.port:
            self._lbl_qs_port.setText(f"Port: {self.ser.port}")
            self._lbl_qs_port.setStyleSheet(self._style_badge(c['accent_gold']))
        else:
            self._lbl_qs_port.setText("Port: Disconnected")
            self._lbl_qs_port.setStyleSheet(self._style_badge(c['danger']))
        # Mode label + value label (RPM gold / DUTY amber)
        if hasattr(self, "_lbl_mode"):
            self._style_mode_value_labels()

    # -- Theme-aware style string helpers -----------------------------------

    def _style_badge(self, text_color: str) -> str:
        """Style string for a quick-status badge label."""
        c = self._colors()
        return (
            f"color: {text_color}; font-weight: bold; "
            f"background-color: {c['bg_console']}; border: 1px solid {c['border']}; "
            "border-radius: 4px; padding: 4px 10px;"
        )

    def _style_connection_button(self):
        """Style the Connect/Disconnect button for the active theme + state."""
        c = self._colors()
        if self.connected:
            self._btn_connect.setStyleSheet(
                f"QPushButton {{ background-color: {c['danger']}; "
                f"color: {c['text']}; }}"
            )
        else:
            self._btn_connect.setStyleSheet(
                f"QPushButton {{ background-color: {c['success']}; "
                f"color: {c['text']}; }}"
            )

    def _style_connection_status(self):
        """Style the * Connected / * Disconnected label."""
        c = self._colors()
        color = c['accent_gold'] if self.connected else c['danger']
        self._lbl_status.setStyleSheet(
            f"color: {color}; font-weight: bold;"
        )

    def _style_console_widgets(self):
        """Re-style the H7 console, GUI console, H7 input, Send button and
        their section labels for the active theme."""
        c = self._colors()

        self._h7_console.setStyleSheet(
            f"QTextEdit {{ background-color: {c['bg_console']}; "
            f"border: 1px solid {c['accent_gold']}; "
            f"border-radius: 4px; color: {c['text']}; "
            "font-family: 'Consolas', 'Courier New', monospace; font-size: 12px; }"
        )

        self._h7_input.setStyleSheet(
            f"QLineEdit {{ background-color: {c['bg_input']}; "
            f"border: 1px solid {c['accent_gold']}; "
            f"border-radius: 4px; padding: 4px 8px; color: {c['text']}; }}"
        )
        # QLineEdit placeholder color is not settable via QSS portably; keep
        # default (handled by palette inherited from Fusion + stylesheet text).

        self._btn_h7_send.setStyleSheet(
            f"QPushButton {{ background-color: {c['bg_input']}; "
            f"border: 1px solid {c['accent_gold']}; "
            f"border-radius: 6px; padding: 4px 14px; color: {c['accent_gold']}; "
            f"font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: {c['selection_bg']}; }}"
        )

        # Keep references to the section labels so they can be re-themed.
        if hasattr(self, "_lbl_h7_console_title"):
            self._lbl_h7_console_title.setStyleSheet(
                f"color: {c['accent_gold']}; font-weight: bold; font-size: 13px;"
            )
        if hasattr(self, "_lbl_gui_console_title"):
            self._lbl_gui_console_title.setStyleSheet(
                f"color: {c['text_muted']}; font-weight: bold; font-size: 13px;"
            )

        self._gui_console.setStyleSheet(
            f"QTextEdit {{ background-color: {c['bg_console']}; "
            f"border: 1px solid {c['border']}; "
            f"border-radius: 4px; color: {c['text_muted']}; "
            "font-family: 'Consolas', 'Courier New', monospace; font-size: 12px; }"
        )

    def _style_help_button(self):
        """Style the GUI Help button for the active theme."""
        c = self._colors()
        self._btn_help.setStyleSheet(
            f"QPushButton {{ background-color: {c['bg_input']}; "
            f"border: 1px solid {c['accent_gold']}; "
            f"color: {c['accent_gold']}; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: {c['selection_bg']}; }}"
        )

    def _style_motor_settings_button(self):
        """Style the Motor State 'Settings' button for the active theme.

        Visually grouped with the help button (same accent/border language)
        so the new entry stays consistent with the existing palette.
        """
        c = self._colors()
        self._btn_motor_settings.setStyleSheet(
            f"QPushButton {{ background-color: {c['bg_input']}; "
            f"border: 1px solid {c['accent_gold']}; "
            f"color: {c['accent_gold']}; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: {c['selection_bg']}; }}"
            f"QPushButton:pressed {{ background-color: {c['pressed_bg']}; }}"
        )

    def _style_mode_value_labels(self):
        """Re-style the Mode label + FB/ROT value labels for the active theme + mode."""
        c = self._colors()
        if self.mode == "RPM":
            mode_color = c['accent_gold']
            value_color = c['accent_gold_bright']
        else:
            mode_color = c['accent_gold_bright']
            value_color = c['accent_gold_bright']
        self._lbl_mode.setStyleSheet(
            f"color: {mode_color}; font-size: 16px; font-weight: bold;"
        )
        self._lbl_fb_value.setStyleSheet(
            f"color: {value_color}; font-size: 18px; font-weight: bold;"
        )
        self._lbl_rot_value.setStyleSheet(
            f"color: {value_color}; font-size: 18px; font-weight: bold;"
        )

    def _style_operating_mode_status(self, cfg: dict):
        """Style the Operating Mode status box.  The semantic background
        (DISARM red / MANUAL amber / AUTONOMOUS green) is preserved across
        themes; only the surrounding border adapts."""
        c = self._colors()
        self._lbl_op_mode_status.setStyleSheet(
            f"QLabel {{ background-color: {cfg['status_bg']}; "
            f"color: {cfg['status_fg']}; "
            f"font-size: 18px; font-weight: bold; "
            f"border: 1px solid {c['border']}; border-radius: 6px; }}"
        )

    def _style_led(self, led: QFrame, color: str | None):
        """Apply active (`color`) or inactive (None) styling to an LED.  The
        inactive colors are theme-aware so dim LEDs read well on both themes."""
        c = self._colors()
        if color is None:
            led.setStyleSheet(
                f"QFrame {{ background-color: {c['led_inactive_bg']}; "
                f"border: 1px solid {c['led_inactive_border']}; "
                "border-radius: 9px; }"
            )
        else:
            led.setStyleSheet(
                f"QFrame {{ background-color: {color}; border: 1px solid {color}; "
                f"border-radius: 9px; }}"
            )

    @staticmethod
    def _make_led() -> QFrame:
        """Small circular LED widget (inactive/dim by default).  Initial
        style is dark; _apply_theme() re-styles it for the active palette."""
        led = QFrame()
        led.setFixedSize(18, 18)
        led.setStyleSheet(
            "QFrame { background-color: #2A2A31; border: 1px solid #3A3A3A; "
            "border-radius: 9px; }"
        )
        return led

    # ======================================================================
    #  Console Logging
    # ======================================================================

    def _log_h7(self, prefix: str, text: str, color: str | None = None):
        """Append a colored line to the H7 Console.  `color` defaults to the
        active theme's text color so newly written lines match the theme."""
        c = self._colors()
        text_color = color if color is not None else c['text']
        ts = time.strftime("%H:%M:%S")
        self._h7_console.append(
            f"<span style='color:{c['accent_gold']};'>[{ts}]</span> "
            f"<span style='color:{text_color};'>{prefix} {text}</span>"
        )
        self._h7_console.moveCursor(QTextCursor.End)

    def _log_gui(self, prefix: str, text: str, color: str | None = None):
        """Append a colored line to the GUI Console.  `color` defaults to the
        active theme's muted-text color so newly written lines match the theme."""
        c = self._colors()
        text_color = color if color is not None else c['text_muted']
        ts = time.strftime("%H:%M:%S")
        self._gui_console.append(
            f"<span style='color:{c['accent_gold']};'>[{ts}]</span> "
            f"<span style='color:{text_color};'>{prefix} {text}</span>"
        )
        self._gui_console.moveCursor(QTextCursor.End)

    def _log_tx(self, cmd: str):
        self._log_h7("[TX-H7]", cmd, self._colors()['accent_gold_bright'])

    def _log_rx(self, text: str):
        self._log_h7("[RX-H7]", text, self._colors()['text'])

    def _log_info(self, text: str):
        self._log_gui("[GUI]", text, self._colors()['text_muted'])

    def _log_err(self, text: str):
        self._log_gui("[GUI-ERROR]", text, self._colors()['danger'])

    def _log_warn(self, text: str):
        self._log_gui("[GUI-WARN]", text, self._colors()['warning'])

    # ======================================================================
    #  Serial Port Management
    # ======================================================================

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

            self._lbl_status.setText("* Connected")
            self._style_connection_status()
            self._btn_connect.setText("Disconnect")
            self._style_connection_button()
            self._port_combo.setEnabled(False)
            self._baud_edit.setEnabled(False)

            self._log_info(f"Connected to {port} @ {baud}")
            self._lbl_qs_port.setText(f"Port: {port}")
            self._lbl_qs_port.setStyleSheet(self._style_badge(self._colors()['accent_gold']))
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
        self._reset_input_state(send_stop=False)
        self.connected = False
        # A pending mode change can never be confirmed while disconnected.
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
        self._lbl_status.setText("* Disconnected")
        self._style_connection_status()
        self._btn_connect.setText("Connect")
        self._style_connection_button()
        self._port_combo.setEnabled(True)
        self._baud_edit.setEnabled(True)
        self._lbl_qs_port.setText("Port: Disconnected")
        self._lbl_qs_port.setStyleSheet(self._style_badge(self._colors()['danger']))

    # ======================================================================
    #  Serial Receive
    # ======================================================================

    def _on_rx_line(self, line: str):
        self._log_rx(line)
        if self._parse_motor_telemetry_line(line):
            pass  # telemetry handled
        else:
            self._parse_rx_for_motor_state(line)
        self._parse_uart_error_line(line)
        self._parse_operating_mode_confirm(line)
        self._parse_imu_line(line)

    def _parse_rx_for_motor_state(self, line: str):
        """Update Link column if link-lost/recovered detected."""
        lower = line.lower()
        link_col = self.MOTOR_COL["link"]
        for tag, row in self.MOTOR_ROW.items():
            if f"link_lost][{tag}" in lower:
                item = self._motor_table.item(row, link_col)
                if item:
                    item.setText("LOST")
                    item.setForeground(QColor(self._colors()["danger"]))
            if f"link_recovered][{tag}" in lower:
                item = self._motor_table.item(row, link_col)
                if item:
                    item.setText("OK")
                    item.setForeground(QColor(self._colors()["success_bright"]))

    # -- UART error / recovery parsing ---------------------------------------
    #
    # The H7 firmware (motor_uart_dma.c) reports UART errors over the
    # terminal link (USART3) as plain log lines.  These are NOT motor
    # protocol frames (ACK/STATUS/FAULT), so the existing table parser
    # ignored them and only the console showed them.  These methods detect
    # those log lines and route them into the motor table's Error column
    # using the firmware's UART->motor mapping.

    def _set_motor_error(self, motor: str, text: str, is_error: bool):
        """Write `text` into the UART error state and re-render the Error column."""
        row = self.MOTOR_ROW.get(motor)
        if row is None:
            return
        self._motor_uart_error_text[motor] = text if is_error else ""
        self._render_motor_error(motor)

    def _render_motor_error(self, motor: str):
        """Render the Error column from UART error + F411 fault code state.

        Priority: UART error > F411 fault code > No Error.
        """
        row = self.MOTOR_ROW.get(motor)
        if row is None:
            return
        c = self._colors()
        col = self.MOTOR_COL["error"]
        item = self._motor_table.item(row, col)
        if item is None:
            return

        uart_err = self._motor_uart_error_text.get(motor, "")
        fc = self._motor_fault_code.get(motor, "0")

        if uart_err:
            item.setText(uart_err)
            item.setForeground(QColor(c["danger"]))
        elif fc != "0":
            item.setText(f"Fault Code: {fc}")
            item.setForeground(QColor(c["danger"]))
        else:
            item.setText("No Error")
            item.setForeground(QColor(c["success_bright"]))

    def _parse_uart_error_line(self, line: str) -> bool:
        """Detect H7 UART error/recovery log lines and update the motor table.

        Returns True when the line was recognized as a UART error/recovery
        line (regardless of whether a matching motor row was found).
        """
        # Raw error-code report (first occurrence or 5 s "still unresolved"
        # repeat).  Decoded bit lines normally follow immediately; until then
        # show the raw code so the table still reflects the error.
        m = _RE_UART_ERROR_CODE.match(line)
        if m:
            uart, code = m.group(1), m.group(2)
            motor = self.UART_TO_MOTOR.get(uart)
            if motor is not None:
                self._uart_report_decoded[uart] = []
                self._set_motor_error(motor, f"UART error code: {code}", is_error=True)
            return True

        # Decoded error: "<CODE> - <Description>" (e.g. "FE - Framing error").
        # Accumulate multiple bits within one report cycle and prefer this
        # richer text over the raw code above.
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

        # RX recovered after a previous UART error -> clear UART error state.
        m = _RE_UART_RECOVERED.match(line)
        if m:
            uart = m.group(1)
            motor = self.UART_TO_MOTOR.get(uart)
            if motor is not None:
                self._uart_report_decoded.pop(uart, None)
                self._set_motor_error(motor, "", is_error=False)
            return True

        return False

    # -- F411 Motor telemetry parsing --------------------------------------

    def _parse_motor_telemetry_line(self, line: str) -> bool:
        """Detect and parse F411 telemetry from [TEL][MOTOR] or legacy [UART_RX].

        Returns True if the line was recognized as telemetry.
        """
        motor = None
        payload = None

        m = _RE_MOTOR_TEL_TAGGED.match(line)
        if m:
            motor = m.group(1)
            payload = m.group(2)
        else:
            m = _RE_MOTOR_TEL_UART.match(line)
            if m:
                uart_tag = m.group(1)
                motor = self.UART_RX_TO_MOTOR.get(uart_tag)
                payload = m.group(2)

        if motor is None or payload is None:
            return False

        tel = self._parse_telemetry_payload(payload)
        if not tel:
            return False

        self._update_motor_telemetry(motor, tel)
        return True

    @staticmethod
    def _parse_telemetry_payload(payload: str) -> dict[str, str]:
        """Parse 'RPM:60,T:0,D:0,...' into {'RPM': '60', 'T': '0', ...}."""
        result = {}
        for token in payload.split(","):
            if ":" not in token:
                continue
            key, val = token.split(":", 1)
            result[key.strip()] = val.strip()
        return result

    def _update_motor_telemetry(self, motor: str, tel: dict[str, str]):
        """Write parsed telemetry values into the motor table row."""
        row = self.MOTOR_ROW.get(motor)
        if row is None:
            return

        c = self._colors()
        col = self.MOTOR_COL
        tbl = self._motor_table
        stored = self._motor_telemetry[motor]

        # Merge new values into stored dict
        for k, v in tel.items():
            stored[k] = v

        # Helper to set a cell
        def _set(col_name: str, text: str, color: str | None = None):
            item = tbl.item(row, col[col_name])
            if item is not None:
                item.setText(text)
                if color:
                    item.setForeground(QColor(color))

        # Direct mappings
        _set("current_rpm", tel.get("RPM", stored.get("RPM", "--")))
        _set("target_rpm", tel.get("T", stored.get("T", "--")))
        _set("drive_duty", tel.get("D", stored.get("D", "--")))
        _set("hall_sensor", tel.get("H", stored.get("H", "--")))
        _set("target_pwm", tel.get("PWM_SET", stored.get("PWM_SET", "--")))
        _set("applied_pwm", tel.get("PWM_ACT", stored.get("PWM_ACT", "--")))
        _set("dropped_commands", tel.get("QDROP", stored.get("QDROP", "--")))
        _set("received_uart_bytes", tel.get("RXB", stored.get("RXB", "--")))

        # Translated fields
        dir_val = tel.get("DIR", stored.get("DIR", "--"))
        _set("direction", self._translate_direction(dir_val))

        app_ph = tel.get("APP_PH", stored.get("APP_PH", "--"))
        ms_text = self._translate_app_phase(app_ph)
        ms_color = None
        if ms_text == "Error":
            ms_color = c["danger"]
        elif ms_text == "Brake":
            ms_color = c["warning"]
        _set("motor_state", ms_text, ms_color)

        sp = tel.get("SP", stored.get("SP", "--"))
        _set("control_mode", self._translate_speed_mode(sp))

        brk = tel.get("BRAKE", stored.get("BRAKE", "--"))
        brk_text = self._translate_brake(brk)
        brk_color = c["danger"] if brk_text == "Brake Active" else None
        _set("brake_status", brk_text, brk_color)

        # Fault code
        fc = tel.get("FC", stored.get("FC", "0"))
        self._motor_fault_code[motor] = fc
        fc_item = tbl.item(row, col["fault_code"])
        if fc_item is not None:
            if fc == "0":
                fc_item.setText("No Error")
                fc_item.setForeground(QColor(c["success_bright"]))
            else:
                fc_item.setText(f"Fault Code: {fc}")
                fc_item.setForeground(QColor(c["danger"]))

        # Re-render Error column (UART error has priority over FC)
        self._render_motor_error(motor)

        # Link -> OK when telemetry received
        link_item = tbl.item(row, col["link"])
        if link_item is not None:
            link_item.setText("OK")
            link_item.setForeground(QColor(c["success_bright"]))

    # -- Telemetry value translators ----------------------------------------

    def _translate_app_phase(self, value: str) -> str:
        return self._APP_PH_MAP.get(value, f"Unknown ({value})")

    def _translate_direction(self, value: str) -> str:
        return self._DIR_MAP.get(value, f"Unknown ({value})")

    def _translate_speed_mode(self, value: str) -> str:
        return self._SP_MAP.get(value, f"Unknown ({value})")

    def _translate_brake(self, value: str) -> str:
        return self._BRAKE_MAP.get(value, f"Unknown ({value})")

    # -- Operating-mode confirmation parsing ---------------------------------
    #
    # The H7 firmware (command_handler.c) prints a confirmation line after it
    # actually applies an operating-mode change:
    #     [MODE] DISARM active, motion commands locked
    #     [MODE] MANUAL active
    #     [MODE] AUTONOMOUS active
    # The GUI treats this line as the single source of truth for the rover's
    # operating mode: the Operating Mode indicator (text + color + LEDs) is
    # updated only here, never optimistically when a mode button is clicked.

    # Map the H7 mode name in the confirmation line to the GUI key.
    _OP_MODE_CONFIRM_TO_KEY = {
        "DISARM": "disarm",
        "MANUAL": "manual",
        "AUTONOMOUS": "auto",
    }

    def _parse_operating_mode_confirm(self, line: str) -> bool:
        """Detect an H7 `[MODE] <NAME> active` confirmation line.

        On a match, marks the mode as confirmed, updates the Operating Mode
        indicator, and clears any pending request.  Returns True when the
        line was recognized as a mode-confirmation line.
        """
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
            self._log_info(f"Operating mode confirmed by H7: {mode_name}")
        elif was_pending is not None:
            # H7 confirmed a mode different from what was requested last, or a
            # mode change was triggered by the firmware itself; reflect it but
            # flag the mismatch to the operator.
            self._log_warn(
                f"H7 confirmed mode {mode_name} (expected "
                f"{self.OPERATING_MODES.get(was_pending, {}).get('label', was_pending)})"
            )
        else:
            self._log_info(f"Operating mode: {mode_name}")
        return True

    def _on_pending_mode_timeout(self):
        """Called when no H7 confirmation arrives for a requested mode change.

        Keeps the previously confirmed Operating Mode indicator unchanged and
        warns the operator that the change was not confirmed by the H7.
        """
        failed = self._pending_mode
        self._pending_mode = None
        if failed is None:
            return
        self._log_warn(
            f"Mode change to "
            f"{self.OPERATING_MODES.get(failed, {}).get('label', failed)} "
            f"not confirmed by H7 - keeping current mode "
            f"({self.OPERATING_MODES.get(self._operating_mode, {}).get('label', self._operating_mode)})."
        )

    # -- IMU telemetry parsing -----------------------------------------------

    def _parse_imu_line(self, line: str) -> bool:
        """Detect and parse MPU_IMU / MPU_CONV_MILLI / MAG_IMU telemetry into the IMU table.

        Supports compact (MPU_IMU), detailed (MPU_CONV_MILLI), and magnetometer (MAG_IMU) formats.
        Unknown extra fields are ignored.  Missing fields keep last value.
        Returns True when the line contained recognized IMU telemetry.
        """
        # Try MPU_IMU first
        fields = _parse_kv_payload(line, "MPU_IMU,")
        is_mpu = True
        if fields is None:
            fields = _parse_kv_payload(line, "MPU_CONV_MILLI,")
            is_mpu = True
            if fields is not None:
                mapped: dict[str, int] = {}
                key_map = {
                    "ACC_X_MG": "AX", "ACC_Y_MG": "AY", "ACC_Z_MG": "AZ",
                    "GYRO_X_MDPS": "GX", "GYRO_Y_MDPS": "GY", "GYRO_Z_MDPS": "GZ",
                    "TEMP_CX100": "TC",
                }
                for src, dst in key_map.items():
                    if src in fields:
                        mapped[dst] = fields[src]
                for k in ("OK", "BIAS", "BSRC", "GFILT", "GDB", "GLPF"):
                    if k in fields:
                        mapped[k] = fields[k]
                fields = mapped if mapped else None

        # Try MAG_IMU
        mag_fields = _parse_kv_payload(line, "MAG_IMU,")
        if mag_fields is not None:
            R = self._IMU_ROW

            # Magnetometer: prefer physical units (UTX100) over raw
            if "MX_UTX100" in mag_fields:
                self._set_imu_cell(R["Mag"], 1, f"{mag_fields['MX_UTX100'] / 100.0:.2f} µT")
            elif "MX" in mag_fields:
                # Fallback: convert raw to µT (3750 LSB/Gauss, 1 Gauss = 100 µT)
                self._set_imu_cell(R["Mag"], 1, f"{mag_fields['MX'] * 100.0 / 3750.0:.2f} µT")

            if "MY_UTX100" in mag_fields:
                self._set_imu_cell(R["Mag"], 2, f"{mag_fields['MY_UTX100'] / 100.0:.2f} µT")
            elif "MY" in mag_fields:
                self._set_imu_cell(R["Mag"], 2, f"{mag_fields['MY'] * 100.0 / 3750.0:.2f} µT")

            if "MZ_UTX100" in mag_fields:
                self._set_imu_cell(R["Mag"], 3, f"{mag_fields['MZ_UTX100'] / 100.0:.2f} µT")
            elif "MZ" in mag_fields:
                self._set_imu_cell(R["Mag"], 3, f"{mag_fields['MZ'] * 100.0 / 3750.0:.2f} µT")

            # Magnetic vector magnitude
            if "BMAG_UTX100" in mag_fields:
                self._set_imu_cell(R["Mag"], 0, f"Mag ({mag_fields['BMAG_UTX100'] / 100.0:.2f} µT)")

            return True

        if fields is None:
            return False

        R = self._IMU_ROW

        # Accel: milli-g -> g, 4 decimal places
        if "AX" in fields:
            self._set_imu_cell(R["Accel"], 1, f"{fields['AX'] / 1000.0:.4f} g")
        if "AY" in fields:
            self._set_imu_cell(R["Accel"], 2, f"{fields['AY'] / 1000.0:.4f} g")
        if "AZ" in fields:
            self._set_imu_cell(R["Accel"], 3, f"{fields['AZ'] / 1000.0:.4f} g")

        # Gyro: milli-dps -> dps, 1 decimal place
        if "GX" in fields:
            self._set_imu_cell(R["Gyro"], 1, f"{fields['GX'] / 1000.0:.1f} dps")
        if "GY" in fields:
            self._set_imu_cell(R["Gyro"], 2, f"{fields['GY'] / 1000.0:.1f} dps")
        if "GZ" in fields:
            self._set_imu_cell(R["Gyro"], 3, f"{fields['GZ'] / 1000.0:.1f} dps")

        # Temperature: centi-Celsius -> Celsius, 1 decimal place, in X column
        if "TC" in fields:
            self._set_imu_cell(R["Temp"], 1, f"{fields['TC'] / 100.0:.1f} °C")

        return True

    # ======================================================================
    #  Serial Send
    # ======================================================================

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
            self._log_h7("[TX-H7]", f"{text}  (not sent - not connected)", self._colors()['warning'])
            self._log_warn("Cannot send to H7: serial port is not connected.")
        else:
            self._send_cmd(text)
        self.setFocus()  # return focus to main window so WASD works again

    # ======================================================================
    #  Mode / Value Management
    # ======================================================================

    def _get_movement_value(self, key: str) -> int:
        """Return the appropriate value for a movement key (W/S=FB, A/D=ROT)."""
        if key in ("W", "S"):
            return self.fb_rpm if self.mode == "RPM" else self.fb_pwm
        else:
            return self.rot_rpm if self.mode == "RPM" else self.rot_pwm

    # -- Operating mode (DISARM / MANUAL / AUTONOMOUS) ---------------------
    #   Distinct from the RPM/DUTY drive mode below.  Commands go through the
    #   same _send_cmd path used by all other H7 terminal commands so that
    #   history/logging/disconnected handling stay consistent.
    def _set_operating_mode(self, mode_key: str):
        """Send an operating-mode command to H7 and wait for confirmation.

        The GUI does NOT optimistically update the Operating Mode indicator
        here.  The H7 serial output is the single source of truth: the
        indicator only changes once a `[MODE] <NAME> active` confirmation line
        is received (see _parse_operating_mode_confirm).  A pending request
        is tracked so a timeout warning can be emitted if H7 does not reply.
        """
        cfg = self.OPERATING_MODES.get(mode_key)
        if cfg is None:
            return
        self._send_cmd(cfg["command"])
        already_pending = self._pending_mode is not None
        self._pending_mode = mode_key
        self._pending_mode_timer.start()
        if not already_pending:
            self._log_info(
                f"Requested {cfg['label']} - waiting for H7 confirmation..."
            )

    def _update_operating_mode_ui(self, mode_key: str):
        """Refresh the three LEDs and the status box for the confirmed mode."""
        cfg = self.OPERATING_MODES.get(mode_key)
        if cfg is None:
            return
        self._operating_mode = mode_key

        # LEDs: only the active one is lit, the rest go dim.
        for key in self.OPERATING_MODE_LED_KEYS:
            led_cfg = self.OPERATING_MODES[key]
            led = {
                "disarm": self._led_red,
                "manual": self._led_yellow,
                "auto":   self._led_green,
            }[key]
            self._style_led(led, led_cfg["led"] if key == mode_key else None)

        # Status box: background + text change with the operating mode.
        self._lbl_op_mode_status.setText(cfg["label"])
        self._style_operating_mode_status(cfg)

    def _set_mode(self, new_mode: str):
        if new_mode == self.mode:
            return
        self.mode = new_mode
        self._lbl_mode.setText(new_mode)
        if new_mode == "RPM":
            self._lbl_fb_label.setText("FB RPM:")
            self._lbl_fb_value.setText(str(self.fb_rpm))
            self._lbl_rot_label.setText("ROT RPM:")
            self._lbl_rot_value.setText(str(self.rot_rpm))
            self._send_cmd("m speed")
        else:
            self._lbl_fb_label.setText("FB PWM:")
            self._lbl_fb_value.setText(str(self.fb_pwm))
            self._lbl_rot_label.setText("ROT PWM:")
            self._lbl_rot_value.setText(str(self.rot_pwm))
            self._send_cmd("m duty")
        # Re-style the Mode + Value labels for the active theme + mode.
        self._style_mode_value_labels()
        self._log_info(f"Mode changed to {new_mode}")
        self._lbl_qs_mode.setText(f"Mode: {new_mode}")
        self._lbl_qs_mode.setStyleSheet(self._style_badge(self._colors()['accent_gold']))

    def _toggle_mode(self):
        self._set_mode("DUTY" if self.mode == "RPM" else "RPM")

    def _adjust_value(self, delta: int, target: str = "fb"):
        """Adjust FB or ROT value by delta. target is 'fb' or 'rot'."""
        if target == "fb":
            if self.mode == "RPM":
                self.fb_rpm = max(0, min(self.RPM_MAX, self.fb_rpm + delta))
                self._lbl_fb_value.setText(str(self.fb_rpm))
                self._log_info(f"FB RPM set to {self.fb_rpm}")
            else:
                d = self.DUTY_STEP if delta > 0 else -self.DUTY_STEP
                self.fb_pwm = max(0, min(self.PWM_MAX, self.fb_pwm + d))
                self._lbl_fb_value.setText(str(self.fb_pwm))
                self._log_info(f"FB PWM set to {self.fb_pwm}")
        else:
            if self.mode == "RPM":
                self.rot_rpm = max(0, min(self.RPM_MAX, self.rot_rpm + delta))
                self._lbl_rot_value.setText(str(self.rot_rpm))
                self._log_info(f"ROT RPM set to {self.rot_rpm}")
            else:
                d = self.DUTY_STEP if delta > 0 else -self.DUTY_STEP
                self.rot_pwm = max(0, min(self.PWM_MAX, self.rot_pwm + d))
                self._lbl_rot_value.setText(str(self.rot_pwm))
                self._log_info(f"ROT PWM set to {self.rot_pwm}")

    # ======================================================================
    #  Movement Command Mapping
    # ======================================================================

    def _movement_cmd(self, key: str) -> str:
        """Return the command string for a movement key in the current mode."""
        val = self._get_movement_value(key)
        if self.mode == "RPM":
            return {"W": f"f{val}", "S": f"b{val}", "A": f"l{val}", "D": f"r{val}"}[key]
        else:
            return {"W": f"fd{val}", "S": f"bd{val}", "A": f"ld{val}", "D": f"rd{val}"}[key]

    # ======================================================================
    #  Keyboard Handling
    # ======================================================================

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
        if key == Qt.Key_Escape:
            return "Escape"
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
        if key == Qt.Key_Plus or text == "+":
            return "NumPlus"
        if key == Qt.Key_Minus or text == "-":
            return "NumMinus"
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
            self._reset_input_state(send_stop=True)
        elif key_id == "X":
            self._send_cmd("brake")
            self._reset_input_state(send_stop=False)
        elif key_id == "M":
            self._toggle_mode()
        elif key_id == "I":
            self._send_cmd("identify")
        elif key_id == "Shift":
            self._active_modifier = "Shift"
            self._adjust_value(self.VALUE_STEP, target="fb")
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "Ctrl":
            self._active_modifier = "Ctrl"
            self._adjust_value(-self.VALUE_STEP, target="fb")
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "NumPlus":
            self._active_modifier = "NumPlus"
            self._adjust_value(self.VALUE_STEP, target="rot")
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "NumMinus":
            self._active_modifier = "NumMinus"
            self._adjust_value(-self.VALUE_STEP, target="rot")
            if not self._repeat_timer.isActive():
                self._repeat_timer.start()
        elif key_id == "Escape":
            self._reset_input_state(send_stop=True)

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
        elif key_id in ("Shift", "Ctrl", "NumPlus", "NumMinus"):
            self._active_modifier = None
            if not self._active_move_key:
                self._repeat_timer.stop()

        super().keyReleaseEvent(event)

    def changeEvent(self, event):
        if event.type() == QEvent.WindowDeactivate:
            self._reset_input_state(send_stop=True)
        super().changeEvent(event)

    def _reset_input_state(self, send_stop: bool = False):
        """Reset all keyboard input state to a clean baseline.

        Call this on focus loss, window deactivate, disconnect, Escape,
        stop, or brake to prevent stale key state from causing runaway
        RPM/PWM changes or phantom movement repeats.
        """
        self._active_modifier = None
        self._active_move_key = None
        self._move_held.clear()
        self._move_order.clear()
        self._keys_held.clear()
        self._repeat_timer.stop()
        self._update_motion_indicator(None)
        if send_stop:
            self._send_cmd("stop")

    def _repeat_movement(self):
        """Called every 500 ms by the repeat timer."""
        # Guard against missed keyReleaseEvent: verify modifier is still held
        if self._active_modifier and self._active_modifier not in self._keys_held:
            self._active_modifier = None
        # Guard against missed keyReleaseEvent: verify movement key is still held
        if self._active_move_key and self._active_move_key not in self._keys_held:
            self._move_held.discard(self._active_move_key)
            self._move_order = deque(k for k in self._move_order if k != self._active_move_key)
            if self._move_order:
                self._active_move_key = self._move_order[-1]
            else:
                self._active_move_key = None

        if self._active_move_key:
            self._send_cmd(self._movement_cmd(self._active_move_key))
        if self._active_modifier == "Shift":
            self._adjust_value(self.VALUE_STEP, target="fb")
        elif self._active_modifier == "Ctrl":
            self._adjust_value(-self.VALUE_STEP, target="fb")
        elif self._active_modifier == "NumPlus":
            self._adjust_value(self.VALUE_STEP, target="rot")
        elif self._active_modifier == "NumMinus":
            self._adjust_value(-self.VALUE_STEP, target="rot")
        if not self._active_move_key and not self._active_modifier:
            self._repeat_timer.stop()

    # ======================================================================
    #  Help Popup
    # ======================================================================

    def _show_help_popup(self):
        c = self._colors()
        dlg = QDialog(self)
        dlg.setWindowTitle("Earendil GUI Help")
        dlg.setMinimumWidth(520)
        dlg.setStyleSheet(f"""
            QDialog {{
                background-color: {c['bg_main']};
                color: {c['text']};
                font-size: 13px;
            }}
            QLabel {{
                color: {c['text']};
            }}
            QPushButton {{
                background-color: {c['bg_input']};
                border: 1px solid {c['accent_gold']};
                border-radius: 6px;
                padding: 8px 24px;
                color: {c['accent_gold']};
                font-weight: bold;
                min-height: 28px;
            }}
            QPushButton:hover {{
                background-color: {c['selection_bg']};
            }}
        """)

        layout = QVBoxLayout(dlg)
        layout.setSpacing(12)

        title = QLabel("Earendil - Rover Control GUI Help")
        title.setStyleSheet(
            f"font-size: 18px; font-weight: bold; color: {c['accent_gold']};"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"color: {c['border']};")
        layout.addWidget(line)

        keys_html = (
            f"<table style='font-size:13px; color:{c['text']};' cellspacing='8'>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>W</b></td><td>Forward (FB value)</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>Space</b></td><td>Stop</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>S</b></td><td>Backward (FB value)</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>X</b></td><td>Brake</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>A</b></td><td>Left (ROT value)</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>M</b></td><td>Toggle RPM/DUTY</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>D</b></td><td>Right (ROT value)</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>I</b></td><td>Identify</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>LShift</b></td><td>FB value +5</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>LCtrl</b></td><td>FB value -5</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>Num+</b></td><td>ROT value +5</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>Num-</b></td><td>ROT value -5</td></tr>"
            f"</table>"
        )
        keys_label = QLabel(keys_html)
        keys_label.setTextFormat(Qt.RichText)
        layout.addWidget(keys_label)

        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet(f"color: {c['border']};")
        layout.addWidget(line2)

        mode_html = (
            f"<table style='font-size:13px; color:{c['text']};' cellspacing='4'>"
            f"<tr><td style='color:{c['accent_gold']};'><b>RPM mode:</b></td>"
            f"<td>W/S use FB RPM, A/D use ROT RPM  (cmd: m speed)</td></tr>"
            f"<tr><td style='color:{c['accent_gold']};'><b>DUTY mode:</b></td>"
            f"<td>W/S use FB PWM, A/D use ROT PWM  (cmd: m duty)</td></tr>"
            f"</table>"
            f"<br>"
            f"<span style='color:{c['text_muted']};'>Held key repeats every 500 ms. "
            f"Shift/Ctrl adjust FB value. Num+/Num- adjust ROT value.</span>"
        )
        mode_label = QLabel(mode_html)
        mode_label.setTextFormat(Qt.RichText)
        layout.addWidget(mode_label)

        line3 = QFrame()
        line3.setFrameShape(QFrame.HLine)
        line3.setStyleSheet(f"color: {c['border']};")
        layout.addWidget(line3)

        console_html = (
            f"<table style='font-size:13px; color:{c['text']};' cellspacing='4'>"
            f"<tr><td style='color:{c['accent_gold']};'><b>H7 Console:</b></td>"
            f"<td>Shows serial TX/RX with the STM32H723</td></tr>"
            f"<tr><td style='color:{c['text_muted']};'><b>GUI Console:</b></td>"
            f"<td>Shows GUI-local messages, warnings, and errors</td></tr>"
            f"</table>"
        )
        console_label = QLabel(console_html)
        console_label.setTextFormat(Qt.RichText)
        layout.addWidget(console_label)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dlg.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignCenter)

        dlg.exec()

    # ======================================================================
    #  F411 Motor Tuning - placeholder command builder + sender
    # ======================================================================
    #  Single source of truth for the placeholder F411 tuning command format.
    #  The GUI forwards tuning commands to the H7 firmware via the
    #  "<MOTOR> <keyword> <args...>" syntax.  The H7 parser validates
    #  and normalises these before forwarding the payload to the
    #  selected F411 motor UART.  For "ALL", the H7 broadcasts to
    #  all four motor UARTs.
    #
    #  Settings dict shape produced by the dialog
    #  (MotorSettingsDialog.collect_f411_tuning_settings):
    #     base      : list[str]*8   (Base PWM 1..8, raw text)
    #     boost     : list[str]*8   (Boost PWM 1..8, raw text)
    #     boostms   : str           (single global Boost MS, raw text)
    #     kick_duty : str           (raw text)
    #     kick_ms   : str           (raw text)
    #     ramp_up   : str           (raw text)
    #     ramp_down : str           (raw text)
    #     kp        : str           (raw text)
    #     ki        : str           (raw text)
    #     telper    : str           (raw text)
    #     custom    : str           (raw text - handled by the dialog itself)

    # H7 motor tuning command keywords - match the firmware parser exactly.
    # Format: "<MOTOR> <keyword> <args...>" sent through _send_cmd().
    F411_TUNE_KW_BASE     = "base"       # <P1>..<P8>
    F411_TUNE_KW_BOOST    = "boost"      # <P1>..<P8> <MS>
    F411_TUNE_KW_KICKDUTY = "kickduty"   # <VALUE>
    F411_TUNE_KW_KICKMS   = "kickms"     # <VALUE>
    F411_TUNE_KW_RAMP     = "ramp"       # <UP> <DOWN>
    F411_TUNE_KW_PI       = "pi"         # <KP> <KI>
    F411_TUNE_KW_TELPER   = "telper"     # <MS>

    def build_f411_tuning_commands(self, target_motor: str, settings: dict) -> list:
        """Build validated H7 motor tuning commands for one motor or ALL.

        `target_motor` is one of "FL", "FR", "RL", "RR", "ALL".
        Returns a list of complete H7 terminal command strings ready to
        send through _send_cmd().  Skips commands whose required fields
        are empty or inconsistent, logging a GUI warning for partial input.
        """
        cmds = []
        log = self._log_warn  # shorthand

        # -- Base PWM: base P1 P2 P3 P4 P5 P6 P7 P8 --------------------
        bases = settings.get("base", [""] * 8) or [""] * 8
        # Pad to 8 if shorter
        while len(bases) < 8:
            bases.append("")
        all_empty = all(v == "" for v in bases[:8])
        some_empty = not all_empty and any(v == "" for v in bases[:8])
        if all_empty:
            pass  # skip silently
        elif some_empty:
            log("[F411-TUNE] Base PWM: not all 8 values filled - skipped")
        else:
            vals = " ".join(v if v != "" else "0" for v in bases[:8])
            cmds.append(f"{target_motor} {self.F411_TUNE_KW_BASE} {vals}")

        # -- Boost PWM: boost P1..P8 MS --------------------------------
        boosts = settings.get("boost", [""] * 8) or [""] * 8
        while len(boosts) < 8:
            boosts.append("")
        boostms = settings.get("boostms", "")
        all_b_empty = all(v == "" for v in boosts[:8]) and boostms == ""
        some_b_empty = not all_b_empty and (
            any(v == "" for v in boosts[:8]) or boostms == "")
        if all_b_empty:
            pass
        elif some_b_empty:
            log("[F411-TUNE] Boost: need all 8 PWM values + Boost MS - skipped")
        else:
            pvals = " ".join(v if v != "" else "0" for v in boosts[:8])
            cmds.append(
                f"{target_motor} {self.F411_TUNE_KW_BOOST} {pvals} {boostms}"
            )

        # -- Kick Duty: kickduty VALUE ----------------------------------
        kick_duty = settings.get("kick_duty", "")
        if kick_duty != "":
            cmds.append(
                f"{target_motor} {self.F411_TUNE_KW_KICKDUTY} {kick_duty}"
            )

        # -- Kick MS: kickms VALUE --------------------------------------
        kick_ms = settings.get("kick_ms", "")
        if kick_ms != "":
            cmds.append(
                f"{target_motor} {self.F411_TUNE_KW_KICKMS} {kick_ms}"
            )

        # -- Ramp: ramp UP DOWN -----------------------------------------
        ramp_up = settings.get("ramp_up", "")
        ramp_dn = settings.get("ramp_down", "")
        if ramp_up == "" and ramp_dn == "":
            pass
        elif ramp_up == "" or ramp_dn == "":
            log("[F411-TUNE] Ramp: need both Up and Down - skipped")
        else:
            cmds.append(
                f"{target_motor} {self.F411_TUNE_KW_RAMP} {ramp_up} {ramp_dn}"
            )

        # -- PI: pi KP KI ----------------------------------------------
        kp = settings.get("kp", "")
        ki = settings.get("ki", "")
        if kp == "" and ki == "":
            pass
        elif kp == "" or ki == "":
            log("[F411-TUNE] PI: need both Kp and Ki - skipped")
        else:
            cmds.append(
                f"{target_motor} {self.F411_TUNE_KW_PI} {kp} {ki}"
            )

        # -- Telemetry Period: telper MS --------------------------------
        telper = settings.get("telper", "")
        if telper != "":
            cmds.append(
                f"{target_motor} {self.F411_TUNE_KW_TELPER} {telper}"
            )

        # -- Custom command ---------------------------------------------
        custom = settings.get("custom", "")
        if custom:
            cmds.append(f"{target_motor} {custom}")

        return cmds

    def send_f411_tuning_command(self, target_motor: str, command: str):
        """Send one validated H7 motor tuning command line and log it.

        `command` must be the full line (e.g. "FL pi 0.8 0.05") and is
        forwarded through _send_cmd() - the same serial path used by all
        other H7 terminal commands.  If serial is not connected, the
        command is still logged with a warning so the operator can see
        what would have been sent.
        with a warning so the operator can see what would have been sent.
        """
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log_warn(
                f"[F411-TUNE] Not sent (serial disconnected): {command}"
            )
            return
        self._log_info(f"[F411-TUNE] {command}")
        self._send_cmd(command)

    def _open_motor_settings(self):
        """Open the F411 Motor Tuning Settings dialog (Modal)."""
        dlg = MotorSettingsDialog(self, self)
        dlg.exec()

    def _open_imu_settings(self):
        """Open the IMU / MAG Settings dialog.

        Prevents duplicate windows by reusing an existing dialog instance.
        If the dialog is already open, it is brought to focus instead of
        creating a new one.
        """
        if self._imu_settings_dialog is not None and self._imu_settings_dialog.isVisible():
            # Dialog is already open, bring it to focus
            self._imu_settings_dialog.raise_()
            self._imu_settings_dialog.activateWindow()
            return

        # Create a new dialog instance
        self._imu_settings_dialog = ImuMagSettingsDialog(self, self)
        self._imu_settings_dialog.show()

    # -- F411 tuning paced-send (QTimer-based, no blocking) --------------

    def enqueue_f411_tuning_sequence(self, commands: list[str], dialog=None):
        """Queue a paced sequence of F411 tuning commands.

        Commands are sent one-by-one via QTimer at TUNING_SEND_INTERVAL_MS
        so the H7 terminal RX FIFO and motor TX FIFO can absorb them
        without loss.  Send buttons are disabled during the sequence and
        re-enabled on completion.
        """
        if self._tuning_send_queue:
            self._log_warn("[F411-TUNE] A tuning sequence is already running")
            return

        self._tuning_dialog_ref = dialog
        self._tuning_send_queue.extend(commands)
        self._log_info(f"[F411-TUNE] Queued {len(commands)} paced command(s)")

        if dialog is not None:
            dialog._set_send_buttons_enabled(False)

        # Send the first command immediately; the timer drives the rest.
        self._send_next_f411_tuning_command()
        if self._tuning_send_queue:
            self._tuning_send_timer.start()

    def _send_next_f411_tuning_command(self):
        """Pop and send the next queued tuning command (timer callback)."""
        if not self._tuning_send_queue:
            self._tuning_send_timer.stop()
            self._log_info("[F411-TUNE] Tuning sequence complete")
            dlg = self._tuning_dialog_ref
            self._tuning_dialog_ref = None
            if dlg is not None:
                dlg._set_send_buttons_enabled(True)
            return

        cmd = self._tuning_send_queue.popleft()
        if not self.connected or not self.ser or not self.ser.is_open:
            self._log_warn(f"[F411-TUNE] Not sent (serial disconnected): {cmd}")
            # Drain remaining queue so buttons get re-enabled.
            self._tuning_send_queue.clear()
            self._tuning_send_timer.stop()
            self._log_warn("[F411-TUNE] Sequence aborted (serial disconnected)")
            dlg = self._tuning_dialog_ref
            self._tuning_dialog_ref = None
            if dlg is not None:
                dlg._set_send_buttons_enabled(True)
            return

        self._log_info(f"[F411-TUNE] {cmd}")
        self._send_cmd(cmd)

    # ======================================================================
    #  Cleanup
    # ======================================================================

    def closeEvent(self, event):
        self._tuning_send_timer.stop()
        self._tuning_send_queue.clear()
        self._repeat_timer.stop()
        self._stop_reader()
        self._close_serial()
        super().closeEvent(event)


# ============================================================================
#  Entry Point
# ============================================================================

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")   # Fusion allows full stylesheet control
    window = EarendilControlGui()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
