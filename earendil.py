#!/usr/bin/env python3
"""
Earendil - Rover Control GUI for STM32H723ZG Main Controller
============================================================
Single-file PySide6 application that connects to the Raspberry Pi
TCP-to-Serial bridge, which forwards commands to the H7 firmware.

Architecture:
    PC (this GUI)  --TCP-->  Raspberry Pi bridge  --Serial-->  STM32H723

Controls:
    W/A/S/D   -> forward / left / backward / right
    T/Y       -> forward-left arc / forward-right arc
    G/H       -> backward-left arc / backward-right arc
    Q/E       -> decrease / increase Turn Ratio
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
        QTabWidget, QCheckBox, QMessageBox, QScrollArea,
    )
    from PySide6.QtCore import Qt, QTimer, QEvent
    from PySide6.QtGui import (
        QFont, QColor, QTextCursor, QKeyEvent, QKeySequence,
        QPainter, QPixmap, QImage,
    )
    from PySide6.QtNetwork import QTcpSocket, QAbstractSocket
except ImportError:
    _missing.append("PySide6  (pip install PySide6)")

if _missing:
    print("ERROR: Missing required packages:")
    for m in _missing:
        print(f"  - {m}")
    sys.exit(1)

# -- TCP defaults -----------------------------------------------------------
DEFAULT_BRIDGE_HOST = "192.168.50.20"
DEFAULT_BRIDGE_PORT = 5000
DEFAULT_CONNECT_TIMEOUT_MS = 5000
MAX_TCP_RX_BUFFER_SIZE = 65536
MAX_TCP_TX_BACKLOG = 65536
CONTROL_HEARTBEAT_PERIOD_MS = 500
LINKSTAT_RETRY_DELAYS_MS = (300, 800, 1500)

# -- Telemetry freshness tracking -------------------------------------------
TELEMETRY_FRESHNESS_CHECK_MS = 500
MOTOR_TELEMETRY_STALE_MS = 2000
IMU_TELEMETRY_STALE_MS = 3000
MAG_TELEMETRY_STALE_MS = 3000
ARM_TELEMETRY_STALE_MS = 3000
DRILL_TELEMETRY_STALE_MS = 3000
SENSOR_ONE_SHOT_TIMEOUT_MS = 3000
IMU_STREAM_AUTODETECT_MIN_PACKETS = 2
IMU_STREAM_AUTODETECT_MAX_GAP_MS = 1000
IMU_ONE_SHOT_AUTODETECT_GUARD_MS = 250


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


def _safe_int(value, default: int = 0) -> int:
    """Best-effort int conversion used by tolerant telemetry parsers."""
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return default


# -- Protocol-boundary normalization ---------------------------------------
_LOG_LEVEL_PREFIX = r"(?:\[(?:INFO|WARN|ERROR|DEBUG|BOOT)\]\s+)?"
_RE_PC_LINK_STATUS = re.compile(
    rf"^{_LOG_LEVEL_PREFIX}PC_LINK,(?P<payload>[A-Z][A-Z0-9_]*:[^\r\n,]*"
    rf"(?:,[A-Z][A-Z0-9_]*:[^\r\n,]*)*)\s*$"
)
_RE_PC_LINK_EVENT = re.compile(
    rf"^{_LOG_LEVEL_PREFIX}\[PC_LINK\]\s+(?P<event>TIMEOUT|RECOVERED)"
    rf"(?:,[^\r\n]*)?\s*$"
)
_RE_ARM_RX_RECORD = re.compile(
    rf"^{_LOG_LEVEL_PREFIX}\[ARM_RX\]\s+(?P<payload>[^\r\n]+?)\s*$"
)
_RE_IMU_STREAM_STATUS = re.compile(
    rf"^{_LOG_LEVEL_PREFIX}IMU_STREAM,EN:(?P<enabled>[01])"
    rf"(?:,PERIOD_MS:\d+)?,OK:1\s*$"
)
_RE_LEADING_ARM = re.compile(r"^(?:arm(?:\s+|$))+", re.IGNORECASE)


def _parse_pc_link_status(line: str) -> str | None:
    """Return ALIVE/TIMEOUT/UNKNOWN for a valid H7 PC-link record.

    The match is deliberately anchored.  A diagnostic or unrelated log line
    that merely contains ``PC_LINK`` is not accepted.
    """
    if not isinstance(line, str):
        return None
    record = line.strip()
    event_match = _RE_PC_LINK_EVENT.fullmatch(record)
    if event_match:
        return "TIMEOUT" if event_match.group("event") == "TIMEOUT" else "ALIVE"

    status_match = _RE_PC_LINK_STATUS.fullmatch(record)
    if not status_match:
        return None
    kv: dict[str, int] = {}
    for item in status_match.group("payload").split(","):
        key, value = item.split(":", 1)
        try:
            kv[key.strip()] = int(value.strip())
        except ValueError:
            continue
    if not kv:
        return None
    if kv.get("TIMEOUT", 0):
        return "TIMEOUT"
    if kv.get("ALIVE", 0):
        return "ALIVE"
    if (kv.get("SEEN", 0) == 0 and kv.get("ALIVE", 0) == 0
            and kv.get("TIMEOUT", 0) == 0):
        return "UNKNOWN"
    return "UNKNOWN"


def _extract_arm_rx_payload(line: str) -> str | None:
    """Extract an F401 payload from one anchored H7 ``[ARM_RX]`` record."""
    if not isinstance(line, str):
        return None
    match = _RE_ARM_RX_RECORD.fullmatch(line.strip())
    if not match:
        return None
    payload = match.group("payload").strip()
    return payload or None


def _parse_imu_stream_status(line: str) -> bool | None:
    """Return the enabled state from one exact H7 IMU stream record.

    Logger prefixes are accepted, but arbitrary lines containing the words
    ``IMU_STREAM`` are deliberately rejected.
    """
    if not isinstance(line, str):
        return None
    match = _RE_IMU_STREAM_STATUS.fullmatch(line.strip())
    if not match:
        return None
    return match.group("enabled") == "1"


def _normalize_manipulation_payload(payload: str) -> str | None:
    """Return an unprefixed, non-empty F401 payload."""
    if not isinstance(payload, str):
        return None
    normalized = payload.strip()
    normalized = _RE_LEADING_ARM.sub("", normalized).strip()
    return normalized or None


def _format_manipulation_command(payload: str) -> str | None:
    """Return exactly one H7 ``arm `` prefix plus the F401 payload."""
    normalized = _normalize_manipulation_payload(payload)
    return f"arm {normalized}" if normalized else None


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
            btn.clicked.connect(lambda _=False, m=motor: self._on_send_to(m))
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
        """Read buttons — one per motor + status label."""
        row = QHBoxLayout()
        row.setSpacing(6)
        self._read_buttons: dict[str, QPushButton] = {}
        for motor in self.MOTOR_TAGS:
            btn = QPushButton(f"Read {motor}")
            btn.clicked.connect(lambda _=False, m=motor: self._on_read_motor(m))
            row.addWidget(btn, 1)
            self._read_buttons[motor] = btn
        self._lbl_read_status = QLabel("")
        self._lbl_read_status.setMinimumWidth(140)
        c = self._gui._colors()
        self._lbl_read_status.setStyleSheet(
            f"color: {c['text_muted']}; font-size: 11px;")
        row.addWidget(self._lbl_read_status)
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

    # ======================================================================
    #  Read config from H7 cache
    # ======================================================================

    def _on_read_motor(self, motor: str):
        """Handle Read FL/FR/RL/RR button click."""
        # Disable read buttons during read
        for btn in self._read_buttons.values():
            btn.setEnabled(False)
        self._set_read_status(motor, f"Reading {motor}...", success=None)
        self._gui.cfgread_start(motor, self)

    def _set_read_status(self, motor: str, text: str, success: bool | None):
        """Update the read status label.

        success=True  -> green, success=False -> red, None -> muted.
        """
        if not hasattr(self, "_lbl_read_status"):
            return
        c = self._gui._colors()
        if success is True:
            color = c.get("success_bright", "#1E8E3E")
        elif success is False:
            color = c.get("danger_bright", "#E02020")
        else:
            color = c.get("text_muted", "#8E8E93")
        self._lbl_read_status.setText(text)
        self._lbl_read_status.setStyleSheet(
            f"color: {color}; font-size: 11px;")
        # Re-enable read buttons after completion or timeout
        if success is not None:
            for btn in self._read_buttons.values():
                btn.setEnabled(True)

    def apply_f411_tuning_config(self, motor: str, cfg: dict):
        """Fill dialog fields from a parsed cfgcache config dict."""
        self._gui._log_info(
            f"[CFGREAD] Applying {motor} config: {cfg}"
        )

        # Kp / Ki — display human values (kp_m / 1000)
        kp_m = cfg.get("kp_m")
        ki_m = cfg.get("ki_m")
        if kp_m is not None:
            self._kp_edit.setText(f"{kp_m / 1000.0:g}")
        if ki_m is not None:
            self._ki_edit.setText(f"{ki_m / 1000.0:g}")

        # Base PWM slots
        base = cfg.get("base")
        if base:
            for i in range(min(len(base), self.NUM_SLOTS)):
                self._base_edits[i].setText(str(base[i]))

        # Boost PWM slots
        boost = cfg.get("boost")
        if boost:
            for i in range(min(len(boost), self.NUM_SLOTS)):
                self._boost_edits[i].setText(str(boost[i]))

        # Boost MS
        if cfg.get("boost_ms") is not None:
            self._boostms_edit.setText(str(cfg["boost_ms"]))

        # Kick Duty / Kick MS
        if cfg.get("kick_duty") is not None:
            self._kick_duty_edit.setText(str(cfg["kick_duty"]))
        if cfg.get("kick_ms") is not None:
            self._kick_ms_edit.setText(str(cfg["kick_ms"]))

        # Ramp Up / Down
        if cfg.get("ramp_up") is not None:
            self._ramp_up_edit.setText(str(cfg["ramp_up"]))
        if cfg.get("ramp_down") is not None:
            self._ramp_dn_edit.setText(str(cfg["ramp_down"]))

        # Telemetry Period
        if cfg.get("telper") is not None:
            self._telper_edit.setText(str(cfg["telper"]))

        # Log actual field values after update
        self._gui._log_info(
            f"[CFGREAD] Applied fields: "
            f"Kp={self._kp_edit.text()} Ki={self._ki_edit.text()} "
            f"BoostMS={self._boostms_edit.text()} "
            f"Base={[e.text() for e in self._base_edits]} "
            f"Boost={[e.text() for e in self._boost_edits]} "
            f"KickDuty={self._kick_duty_edit.text()} "
            f"KickMS={self._kick_ms_edit.text()} "
            f"RampUp={self._ramp_up_edit.text()} "
            f"RampDown={self._ramp_dn_edit.text()} "
            f"TelPer={self._telper_edit.text()}"
        )

    def closeEvent(self, event):
        """Clean up pending cfgread when dialog is closed."""
        if self._gui._cfgread_dialog is self:
            self._gui._cfgread_cleanup()
        super().closeEvent(event)


# ============================================================================
#  IMU / MAG Settings Dialog
# ============================================================================

class ImuMagSettingsDialog(QDialog):
    """IMU / MAG Settings dialog — compact control panel."""

    C_BG       = "#20232b"
    C_CARD     = "#2a2e3d"
    C_BORDER   = "#4c5368"
    C_TITLE    = "#7fb4ff"
    C_TEXT     = "#d7dbe6"
    C_TEXT_DIM = "#9aa3b5"
    C_GREEN    = "#4dbb74"
    C_RED      = "#d05a5a"
    C_BTN_BG   = "#3a3f52"
    C_BTN_HOV  = "#46506a"
    C_BTN_PRS  = "#555b72"
    C_INPUT_BG = "#353a4d"

    def __init__(self, main_gui: "EarendilControlGui", parent=None):
        super().__init__(parent)
        self._gui = main_gui
        self.setWindowTitle("IMU / MAG Settings")
        self.setMinimumSize(660, 560)
        self._apply_theme_style()

        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(14, 14, 14, 14)

        grid = QGridLayout()
        grid.setSpacing(10)
        grid.addWidget(self._build_sensor_detection_card(), 0, 0)
        grid.addWidget(self._build_initialization_card(),   0, 1)
        grid.addWidget(self._build_mpu_read_test_card(),    1, 0)
        grid.addWidget(self._build_stream_control_card(),   1, 1)
        grid.addWidget(self._build_runtime_tuning_card(),   2, 0)
        grid.addWidget(self._build_bias_card(),             2, 1)
        grid.addWidget(self._build_mag_controls_card(),     3, 0)
        grid.addWidget(self._build_quick_help_card(),       3, 1)
        root.addLayout(grid, 1)

        root.addLayout(self._build_bottom_bar())

    # ── helpers ──────────────────────────────────────────────────────────

    def _card(self, title: str) -> tuple[QGroupBox, QVBoxLayout]:
        grp = QGroupBox(title)
        grp.setStyleSheet(f"""
            QGroupBox {{
                background-color: {self.C_CARD};
                border: 1px solid {self.C_BORDER};
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 12px;
                padding-bottom: 6px;
                font-weight: bold;
                font-size: 12px;
                color: {self.C_TITLE};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: {self.C_TITLE};
                font-size: 12px;
            }}
        """)
        lay = QVBoxLayout(grp)
        lay.setSpacing(6)
        lay.setContentsMargins(10, 6, 10, 10)
        return grp, lay

    def _btn(self, label: str, cmd: str, *, primary=False, danger=False) -> QPushButton:
        b = QPushButton(label)
        b.setFixedHeight(34)
        b.setMinimumWidth(84)
        b.setStyleSheet(self._bs(primary, danger))
        b.clicked.connect(lambda _, c=cmd: self._gui._send_cmd(c))
        return b

    def _action_btn(self, label: str, callback, *, primary=False,
                    danger=False) -> QPushButton:
        """Build a button whose action owns protocol-side state changes."""
        b = QPushButton(label)
        b.setFixedHeight(34)
        b.setMinimumWidth(84)
        b.setStyleSheet(self._bs(primary, danger))
        b.clicked.connect(callback)
        return b

    def _bs(self, primary=False, danger=False) -> str:
        if primary:
            bg, hov, prs, bd, fg = "#3a6b3f", "#4a8b50", "#2a5b30", "#5a9b60", "#b8f0bb"
        elif danger:
            bg, hov, prs, bd, fg = "#5a3030", "#7a4040", "#4a2020", "#8a5050", "#e8b0b0"
        else:
            bg, hov, prs, bd, fg = self.C_BTN_BG, self.C_BTN_HOV, self.C_BTN_PRS, self.C_BORDER, self.C_TEXT
        return (f"QPushButton{{background:{bg};border:1px solid {bd};border-radius:5px;"
                f"padding:4px 10px;color:{fg};font-size:11px;font-weight:bold;}}"
                f"QPushButton:hover{{background:{hov};}}"
                f"QPushButton:pressed{{background:{prs};}}")

    def _hint(self, text: str) -> QLabel:
        l = QLabel(text)
        l.setStyleSheet(f"color:{self.C_TEXT_DIM};font-size:10px;border:none;")
        l.setWordWrap(True)
        return l

    def _spin_row(self, label: str, spin: QSpinBox, unit: str, cb) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(6)
        row.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(label)
        lbl.setFixedWidth(120)
        lbl.setStyleSheet(f"color:{self.C_TEXT};font-size:11px;border:none;")
        row.addWidget(lbl)
        spin.setFixedHeight(32)
        spin.setFixedWidth(80)
        spin.setStyleSheet(f"QSpinBox{{background:{self.C_INPUT_BG};border:1px solid {self.C_BORDER};"
                           f"border-radius:4px;padding:3px 6px;color:{self.C_TEXT};font-size:11px;}}")
        row.addWidget(spin)
        u = QLabel(unit)
        u.setFixedWidth(40)
        u.setStyleSheet(f"color:{self.C_TEXT_DIM};font-size:11px;border:none;")
        row.addWidget(u)
        s = QPushButton("Set")
        s.setFixedSize(64, 32)
        s.setStyleSheet(self._bs(primary=True))
        s.clicked.connect(cb)
        row.addWidget(s)
        row.addStretch()
        return row

    # ── card builders ────────────────────────────────────────────────────

    def _build_sensor_detection_card(self) -> QGroupBox:
        grp, lay = self._card("Sensor Detection")
        r = QHBoxLayout(); r.setSpacing(8)
        r.addWidget(self._btn("I2C Scan", "i2cscan"))
        r.addWidget(self._btn("MPU WHO", "mpuwho"))
        r.addWidget(self._btn("MAG WHO", "magwho"))
        lay.addLayout(r)
        lay.addWidget(self._hint("Check I2C bus and sensor identity"))
        return grp

    def _build_initialization_card(self) -> QGroupBox:
        grp, lay = self._card("Initialization")
        r = QHBoxLayout(); r.setSpacing(8)
        r.addWidget(self._btn("MPU Init", "mpuinit"))
        r.addWidget(self._btn("MAG Init", "maginit"))
        b = self._btn("Init All", "", primary=True)
        b.clicked.disconnect(); b.clicked.connect(self._init_all)
        r.addWidget(b)
        lay.addLayout(r)
        lay.addWidget(self._hint("Initialize MPU, MAG, or both sensors"))
        return grp

    def _build_mpu_read_test_card(self) -> QGroupBox:
        grp, lay = self._card("MPU Read / Test")
        r1 = QHBoxLayout(); r1.setSpacing(8)
        r1.addWidget(QLabel("Read:"))
        r1.addWidget(self._action_btn(
            "Raw", lambda: self._gui._request_imu_one_shot("mpuraw")))
        r1.addWidget(self._action_btn(
            "Conv", lambda: self._gui._request_imu_one_shot("mpuconv")))
        r1.addStretch()
        lay.addLayout(r1)
        r2 = QHBoxLayout(); r2.setSpacing(8)
        r2.addWidget(QLabel("Test:")); r2.addWidget(self._btn("Config", "mpucfgtest"))
        r2.addWidget(self._btn("Gyro", "mpugyrotest")); r2.addStretch()
        lay.addLayout(r2)
        for l in (r1.itemAt(0).widget(), r2.itemAt(0).widget()):
            l.setFixedWidth(40)
            l.setStyleSheet(f"color:{self.C_TEXT_DIM};font-size:11px;font-weight:bold;border:none;")
        return grp

    def _build_stream_control_card(self) -> QGroupBox:
        grp, lay = self._card("Stream Control")
        r = QHBoxLayout(); r.setSpacing(8)
        r.addWidget(self._action_btn(
            "Stream ON", self._gui._request_imu_stream_on, primary=True))
        r.addWidget(self._action_btn(
            "Stream OFF", self._gui._request_imu_stream_off, danger=True))
        self._lbl_stream_chip = QLabel("UNKNOWN")
        self._lbl_stream_chip.setStyleSheet(self._chip_style(self.C_TEXT_DIM))
        self._lbl_stream_chip.setFixedWidth(78)
        self._lbl_stream_chip.setAlignment(Qt.AlignCenter)
        r.addWidget(self._lbl_stream_chip)
        r.addStretch()
        lay.addLayout(r)
        return grp

    def _build_runtime_tuning_card(self) -> QGroupBox:
        grp, lay = self._card("Runtime Tuning")
        self._spin_telper = QSpinBox(); self._spin_telper.setRange(20, 5000); self._spin_telper.setValue(100)
        lay.addLayout(self._spin_row("Telemetry Period:", self._spin_telper, "ms", self._set_telper))
        self._spin_deadband = QSpinBox(); self._spin_deadband.setRange(0, 2000); self._spin_deadband.setValue(250)
        lay.addLayout(self._spin_row("Gyro Deadband:", self._spin_deadband, "mdps", self._set_deadband))
        self._spin_lpf = QSpinBox(); self._spin_lpf.setRange(1, 1000); self._spin_lpf.setValue(250)
        lay.addLayout(self._spin_row("Gyro LPF:", self._spin_lpf, "‰", self._set_lpf))
        lay.addSpacing(4)
        r4 = QHBoxLayout(); r4.setSpacing(8)
        r4.addWidget(QLabel("Gyro Filter:"))
        r4.addWidget(self._btn("Status", "imu gyrofilter status"))
        r4.addWidget(self._btn("ON", "imu gyrofilter on", primary=True))
        r4.addWidget(self._btn("OFF", "imu gyrofilter off", danger=True))
        self._lbl_gyro_filter_chip = QLabel("--")
        self._lbl_gyro_filter_chip.setStyleSheet(self._chip_style(self.C_TEXT_DIM))
        self._lbl_gyro_filter_chip.setFixedWidth(50)
        self._lbl_gyro_filter_chip.setAlignment(Qt.AlignCenter)
        r4.addWidget(self._lbl_gyro_filter_chip)
        r4.addStretch()
        lay.addLayout(r4)
        r4.itemAt(0).widget().setStyleSheet(f"color:{self.C_TEXT};font-size:11px;font-weight:bold;border:none;")
        return grp

    def _build_bias_card(self) -> QGroupBox:
        grp, lay = self._card("Bias / Calibration")
        r = QHBoxLayout(); r.setSpacing(8)
        r.addWidget(self._btn("Status", "mpubias"))
        r.addWidget(self._btn("Enable", "mpubiason", primary=True))
        r.addWidget(self._btn("Disable", "mpubiasoff", danger=True))
        r.addWidget(self._btn("Clear", "mpubiasclear", danger=True))
        r.addStretch()
        lay.addLayout(r)
        lay.addWidget(self._hint("Use after sensor is stable and stationary."))
        return grp

    def _build_mag_controls_card(self) -> QGroupBox:
        grp, lay = self._card("MAG Controls")
        r = QHBoxLayout(); r.setSpacing(8)
        r.addWidget(self._action_btn(
            "MAG Raw", lambda: self._gui._request_mag_one_shot("magraw")))
        r.addWidget(self._action_btn(
            "MAG µT", lambda: self._gui._request_mag_one_shot("magimu")))
        r.addWidget(self._btn("MAG Help", "maghelp"))
        r.addStretch()
        lay.addLayout(r)
        lay.addWidget(self._hint("Read raw magnetometer or converted µT values."))
        return grp

    def _build_quick_help_card(self) -> QGroupBox:
        grp, lay = self._card("Quick Help")
        items = [
            ("Scan", "find I2C devices"),
            ("WHO", "read sensor identity"),
            ("Raw / Conv", "read sensor values"),
            ("Stream", "periodic telemetry"),
            ("Tuning", "runtime filter settings"),
        ]
        for name, desc in items:
            r = QHBoxLayout(); r.setSpacing(6); r.setContentsMargins(0, 0, 0, 0)
            n = QLabel(name); n.setFixedWidth(80)
            n.setStyleSheet(f"color:{self.C_TEXT_DIM};font-size:11px;font-weight:bold;border:none;")
            r.addWidget(n)
            d = QLabel(desc)
            d.setStyleSheet(f"color:{self.C_TEXT_DIM};font-size:11px;border:none;")
            r.addWidget(d, 1)
            lay.addLayout(r)
        lay.addSpacing(4)
        r = QHBoxLayout()
        r.addWidget(self._btn("IMU Help", "imu help"))
        r.addStretch()
        lay.addLayout(r)
        return grp

    def _build_bottom_bar(self) -> QHBoxLayout:
        bar = QHBoxLayout(); bar.setSpacing(10)
        info = QLabel("Recommended flow: Scan → WHO → Init → Read/Stream")
        info.setStyleSheet(f"color:{self.C_TEXT_DIM};font-size:11px;font-style:italic;border:none;")
        bar.addWidget(info, 1)
        btn = QPushButton("Close"); btn.setFixedSize(120, 34)
        btn.setStyleSheet(self._bs()); btn.clicked.connect(self.close)
        bar.addWidget(btn)
        return bar

    def _chip_style(self, color: str) -> str:
        return (f"color:{color};font-size:10px;font-weight:bold;"
                f"background:{self.C_BTN_BG};border:1px solid {self.C_BORDER};"
                f"border-radius:10px;padding:2px 8px;")

    # ── actions ──────────────────────────────────────────────────────────

    def _init_all(self):
        self._gui._send_cmd("mpuinit")
        sid = self._gui._tcp_session_id
        QTimer.singleShot(200, lambda sid=sid: (
            self._gui._send_cmd("maginit") if self._gui._is_current_tcp_session(sid) else None
        ))

    def _set_telper(self):
        self._gui._send_cmd(f"imu telper {self._spin_telper.value()}")

    def _set_deadband(self):
        self._gui._send_cmd(f"imu deadband {self._spin_deadband.value()}")

    def _set_lpf(self):
        self._gui._send_cmd(f"imu lpf {self._spin_lpf.value()}")

    # ── theme ────────────────────────────────────────────────────────────

    def _apply_theme_style(self):
        self.setStyleSheet(f"""
            QDialog{{background:{self.C_BG};color:{self.C_TEXT};}}
            QLabel{{color:{self.C_TEXT};font-size:11px;border:none;}}
        """)

    # ── status updates ───────────────────────────────────────────────────

    def update_stream_status(self, status: str):
        if hasattr(self, '_lbl_stream_chip'):
            normalized = status.upper()
            if normalized == "ON":
                c = self.C_GREEN
            elif normalized in ("STARTING", "STOPPING"):
                c = "#e0ad4f"
            elif normalized == "STALE":
                c = self.C_RED
            else:
                c = self.C_TEXT_DIM
            self._lbl_stream_chip.setText(normalized)
            self._lbl_stream_chip.setStyleSheet(self._chip_style(c))

    def update_gyro_filter_status(self, status: str):
        if hasattr(self, '_lbl_gyro_filter_chip'):
            c = self.C_GREEN if "ON" in status.upper() else self.C_TEXT_DIM
            self._lbl_gyro_filter_chip.setText(status)
            self._lbl_gyro_filter_chip.setStyleSheet(self._chip_style(c))

    def closeEvent(self, event):
        event.accept()


# ============================================================================
#  Manipulation Arm Settings Dialog
#  Five-tab settings window for the F401 manipulation arm controller.
#  Commands are built from the dialog fields and sent through the existing
#  EarendilControlGui._send_cmd() serial path, so logging / disconnected
#  handling stays consistent with the rest of the GUI.  This dialog only
#  sends commands when the user explicitly clicks a Send / Test / Action
#  button; it never sends movement commands automatically on open or close.
# ============================================================================

class ManipulationArmSettingsDialog(QDialog):
    """Manipulation Arm Settings dialog (5 tabs).

    Provides editing / sending of manipulation-only F401 settings through
    the existing ``_send_cmd()`` serial path.  Sections:

        1. Axis Settings       - J1..J6: invert, maxpwm, default, gain, dead, stopmode
        2. Position Control    - J1..J3: kp, kd, minpwm, tol, min/max angle, limits, brakepoint
        3. Sensor / AS5600     - J1..J3: AS5600 channel assignment, zero / read actions
        4. Joystick / Keybind  - J1..J6: keybind forward/back + test buttons
        5. System / Save       - mode / stop / heartbeat / params / save with confirmation

    The dialog reuses the active theme via ``self._gui._colors()`` so it stays
    readable in both light and dark themes.  ``params`` parsing is performed
    by the main GUI (``_arm_parse_params``) and dispatched here through
    ``apply_params`` whenever the dialog is open.
    """

    # -- Manipulation joint inventory --------------------------------------
    ARM_JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
    ARM_JOINT_LABELS = {
        "J1": "J1 Base",
        "J2": "J2 Shoulder",
        "J3": "J3 Elbow",
        "J4": "J4 Wrist Pitch",
        "J5": "J5 Wrist Twist",
        "J6": "J6 Gripper",
    }
    # Numeric joint id spoken by the F401 ``set <j> ...`` parser.
    ARM_JOINT_NUM = {"J1": 1, "J2": 2, "J3": 3, "J4": 4, "J5": 5, "J6": 6}
    # J1..J3 support position control + AS5600.  J4..J6 do not (by spec).
    ARM_HAS_POSITION = {"J1": True, "J2": True, "J3": True,
                        "J4": False, "J5": False, "J6": False}
    ARM_HAS_AS5600 = ARM_HAS_POSITION
    # Stop modes available per joint.  J1..J3: coast/brake/hold/hybrid.
    # J4..J6: coast/brake only - hold/hybrid are not offered unless firmware
    # explicitly supports them; this keeps the GUI conservative by default.
    STOPMODES_FULL = ("coast", "brake", "hold", "hybrid")
    STOPMODES_LIMITED = ("coast", "brake")
    AS5600_CHANNELS = ("OFF", "0", "1", "2", "3", "4", "5", "6", "7")
    BRAKEPOINT_OFF = "OFF"

    def __init__(self, main_gui: "EarendilControlGui", parent=None):
        super().__init__(parent)
        self._gui = main_gui
        self.setWindowTitle("Manipulation Arm Settings")
        self.setMinimumSize(900, 580)
        self._apply_theme_style()

        # Local "dirty" flag - set True by any field edit, cleared on save
        # success (OK SAVED_FLASH) or Refresh from params / Save Settings.
        self._dirty: bool = False

        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        # -- Status bar -----------------------------------------------------
        self._lbl_status = QLabel("")
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setStyleSheet(self._muted_style())
        root.addWidget(self._lbl_status)

        # -- Tabs -----------------------------------------------------------
        self._tabs = QTabWidget()
        self._tabs.addTab(self._build_axis_tab(),         "Axis Settings")
        self._tabs.addTab(self._build_position_tab(),     "Position Control")
        self._tabs.addTab(self._build_sensor_tab(),       "Sensor / AS5600")
        self._tabs.addTab(self._build_joystick_tab(),    "Joystick / Keybind")
        self._tabs.addTab(self._build_system_tab(),       "System / Save")
        root.addWidget(self._tabs, 1)

        # -- Bottom bar -----------------------------------------------------
        root.addLayout(self._build_bottom_bar())

        self.set_status("Ready. Edit fields, then Send per row.", None)

    # ======================================================================
    #  Theme / utility helpers
    # ======================================================================

    def _apply_theme_style(self):
        c = self._gui._colors()
        self.setStyleSheet(f"""
            QDialog {{
                background-color: {c['bg_main']};
                color: {c['text']};
                font-size: 13px;
                font-weight: {c['font_weight']};
            }}
            QLabel {{ color: {c['text']}; }}
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
                border-radius: 5px;
                padding: 4px 10px;
                color: {c['accent_gold']};
                font-weight: bold;
                min-height: 24px;
            }}
            QPushButton:hover {{ background-color: {c['selection_bg']}; }}
            QPushButton:pressed {{ background-color: {c['pressed_bg']}; }}
            QPushButton:disabled {{ color: {c['text_muted']};
                                    border-color: {c['border']}; }}
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
                background-color: {c['bg_input']};
                border: 1px solid {c['border']};
                border-radius: 4px;
                padding: 3px 6px;
                color: {c['text']};
                font-weight: {c['font_weight']};
            }}
            QTableWidget {{
                background-color: {c['bg_table']};
                border: 1px solid {c['border']};
                gridline-color: {c['gridline']};
                color: {c['text']};
                font-size: 12px;
            }}
            QTableWidget::item {{ padding: 2px; }}
            QHeaderView::section {{
                background-color: {c['table_header']};
                color: {c['accent_gold']};
                border: none;
                border-right: 1px solid {c['border']};
                border-bottom: 1px solid {c['border']};
                padding: 4px;
                font-weight: bold;
            }}
            QTabWidget::pane {{
                border: 1px solid {c['border']};
                background-color: {c['bg_main']};
            }}
            QTabBar::tab {{
                background-color: {c['bg_input']};
                color: {c['text']};
                border: 1px solid {c['border']};
                padding: 6px 14px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: {c['selection_bg']};
                color: {c['accent_gold']};
                border-color: {c['accent_gold']};
            }}
            QCheckBox {{ color: {c['text']}; }}
            QCheckBox::indicator {{
                width: 14px; height: 14px;
                border: 1px solid {c['border']};
                background: {c['bg_input']};
            }}
            QCheckBox::indicator:checked {{
                background: {c['accent_gold']};
                border-color: {c['accent_gold']};
            }}
        """)

    def _muted_style(self) -> str:
        c = self._gui._colors()
        return f"color: {c['text_muted']}; font-size: 11px; border: none;"

    def _hint_label(self, text: str) -> QLabel:
        l = QLabel(text)
        l.setWordWrap(True)
        l.setStyleSheet(self._muted_style())
        return l

    def _btn(self, label: str, *, primary=False, danger=False) -> QPushButton:
        b = QPushButton(label)
        c = self._gui._colors()
        bg = c["bg_input"]
        hov = c["selection_bg"]
        prs = c["pressed_bg"]
        bd = c["accent_gold"]
        fg = c["accent_gold"]
        if primary:
            bg = c.get("success", "#1e6e3e")
            bd = c.get("success_bright", "#3CB371")
            fg = c.get("success_bright", "#3CB371")
        elif danger:
            bd = c["danger_bright"]
            fg = c["danger_bright"]
        b.setStyleSheet(
            f"QPushButton {{ background-color: {bg}; border: 1px solid {bd};"
            f" border-radius: 5px; padding: 4px 10px; color: {fg};"
            f" font-weight: bold; min-height: 24px; }}"
            f" QPushButton:hover {{ background-color: {hov}; }}"
            f" QPushButton:pressed {{ background-color: {prs}; }}"
        )
        return b

    def _mark_dirty(self, *_):
        """Mark local settings as edited / unsaved and update the status hint."""
        if not self._dirty:
            self._dirty = True
            c = self._gui._colors()
            self._lbl_status.setStyleSheet(
                f"color: {c['warning']}; font-size: 11px; border: none;")
            self.set_status("\u26A0 RAM settings edited but not saved.", "warn")

    def set_status(self, text: str, level: str | None = None):
        """Update the dialog status label.

        level: None (muted), "warn" (warning), "err" (error),
               "ok" (success), "dirty" (visible dir
 
        Also drives the local dirty flag visibility for the operator.
        """
        c = self._gui._colors()
        if level == "warn":
            color = c["warning"]
        elif level == "err":
            color = c["danger_bright"]
        elif level == "ok":
            color = c["success_bright"]
        else:
            color = c["text_muted"]
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(
            f"color: {color}; font-size: 11px; border: none;")

    def clear_dirty(self):
        """Mark local settings as saved / clean."""
        self._dirty = False
        self.set_status("Settings saved to flash.", "ok")

    def is_dirty(self) -> bool:
        return self._dirty

    # ======================================================================
    #  SECTION 1 - Axis Settings tab
    # ======================================================================

    def _build_axis_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(6)
        lay.setContentsMargins(8, 8, 8, 8)

        headers = ["Axis", "Invert", "MaxPWM", "DefaultPWM",
                   "PosGain", "NegGain", "Dead", "StopMode", "Send"]
        self._axis_table = QTableWidget(len(self.ARM_JOINTS), len(headers))
        self._axis_table.setHorizontalHeaderLabels(headers)
        self._axis_table.verticalHeader().setVisible(False)
        h = self._axis_table.horizontalHeader()
        if h:
            h.setSectionResizeMode(0, QHeaderView.Stretch)
            for col in range(1, len(headers) - 1):
                h.setSectionResizeMode(col, QHeaderView.ResizeToContents)
            h.setSectionResizeMode(len(headers) - 1, QHeaderView.ResizeToContents)

        self._axis_widgets: dict[str, dict] = {}
        for row, joint in enumerate(self.ARM_JOINTS):
            self._axis_table.setCellWidget(row, 0, self._static_label(self.ARM_JOINT_LABELS[joint]))

            w_invert = QCheckBox()
            w_invert.stateChanged.connect(self._mark_dirty)
            self._axis_table.setCellWidget(row, 1, w_invert)

            w_maxpwm = self._make_spin(0, 255, 200)
            w_default = self._make_spin(0, 255, 100)
            w_posgain = self._make_spin(100, 3000, 1000)
            w_neggain = self._make_spin(100, 3000, 1000)
            w_dead = self._make_spin(0, 400, 50)
            self._axis_table.setCellWidget(row, 2, w_maxpwm)
            self._axis_table.setCellWidget(row, 3, w_default)
            self._axis_table.setCellWidget(row, 4, w_posgain)
            self._axis_table.setCellWidget(row, 5, w_neggain)
            self._axis_table.setCellWidget(row, 6, w_dead)

            stop_modes = (self.STOPMODES_FULL if joint in ("J1", "J2", "J3")
                          else self.STOPMODES_LIMITED)
            w_stopmode = QComboBox()
            w_stopmode.addItems(stop_modes)
            w_stopmode.setCurrentIndex(0)
            w_stopmode.currentIndexChanged.connect(self._mark_dirty)
            self._axis_table.setCellWidget(row, 7, w_stopmode)

            btn = self._btn("Send")
            btn.clicked.connect(lambda _=False, j=joint: self._send_axis_row(j))
            self._axis_table.setCellWidget(row, 8, btn)

            self._axis_widgets[joint] = {
                "invert":   w_invert,
                "maxpwm":   w_maxpwm,
                "default":  w_default,
                "posgain":  w_posgain,
                "neggain":  w_neggain,
                "dead":     w_dead,
                "stopmode": w_stopmode,
            }

        lay.addWidget(self._axis_table)

        row = QHBoxLayout()
        b_all = self._btn("Send All Axis Settings", primary=True)
        b_all.clicked.connect(self._send_all_axis)
        b_refresh = self._btn("Refresh from params")
        b_refresh.clicked.connect(self._refresh_params)
        row.addWidget(b_all)
        row.addWidget(b_refresh)
        row.addStretch()
        lay.addLayout(row)
        lay.addWidget(self._hint_label(
            "Sends one ``set <j> <field> <value>`` F401 command per changed "
            "field through _send_cmd().  J1..J3 may use hold/hybrid; "
            "J4..J6 expose coast/brake only by default."))
        return tab

    def _send_axis_row(self, joint: str):
        """Send the changed / current row fields as ``set <j> ...`` commands."""
        w = self._axis_widgets.get(joint)
        if not w:
            return
        n = self.ARM_JOINT_NUM[joint]
        cmds = [
            f"set {n} invert {1 if w['invert'].isChecked() else 0}",
            f"set {n} maxpwm {w['maxpwm'].value()}",
            f"set {n} default {w['default'].value()}",
            f"set {n} posgain {w['posgain'].value()}",
            f"set {n} neggain {w['neggain'].value()}",
            f"set {n} dead {w['dead'].value()}",
            f"set {n} stopmode {w['stopmode'].currentText()}",
        ]
        self._gui.send_arm_setting_sequence(cmds)
        self._mark_dirty()

    def _send_all_axis(self):
        for joint in self.ARM_JOINTS:
            self._send_axis_row(joint)

    def _refresh_params(self):
        """Send ``params`` to request full F401 settings dump."""
        self._gui.send_arm_setting_command("params")

    # ======================================================================
    #  SECTION 2 - Position Control tab (J1..J3)
    # ======================================================================

    def _build_position_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(6)
        lay.setContentsMargins(8, 8, 8, 8)

        position_joints = ("J1", "J2", "J3")
        headers = ["Axis", "KP", "KD", "MinPWM", "TolDeg",
                   "MinAngle", "MaxAngle", "Limits", "Brakepoint", "Send"]
        self._pos_table = QTableWidget(len(position_joints), len(headers))
        self._pos_table.setHorizontalHeaderLabels(headers)
        self._pos_table.verticalHeader().setVisible(False)
        h = self._pos_table.horizontalHeader()
        if h:
            h.setSectionResizeMode(0, QHeaderView.Stretch)
            for col in range(1, len(headers)):
                h.setSectionResizeMode(col, QHeaderView.ResizeToContents)

        self._pos_widgets: dict[str, dict] = {}
        for row, joint in enumerate(position_joints):
            self._pos_table.setCellWidget(row, 0,
                                          self._static_label(self.ARM_JOINT_LABELS[joint]))
            w_kp = self._make_spin(0, 10000, 800)
            w_kd = self._make_spin(0, 10000, 40)
            w_minpwm = self._make_spin(0, 255, 60)
            w_tol = self._make_dspin(0.1, 20.0, 2.0, 1)
            w_min = self._make_dspin(-360.0, 360.0, -90.0, 1)
            w_max = self._make_dspin(-360.0, 360.0, 90.0, 1)
            w_limits = QCheckBox()
            w_limits.setChecked(True)
            w_limits.stateChanged.connect(self._mark_dirty)
            w_bp = QLineEdit()
            w_bp.setPlaceholderText("angle or OFF")
            w_bp.setFixedWidth(80)
            w_bp.textEdited.connect(self._mark_dirty)
            self._pos_table.setCellWidget(row, 1, w_kp)
            self._pos_table.setCellWidget(row, 2, w_kd)
            self._pos_table.setCellWidget(row, 3, w_minpwm)
            self._pos_table.setCellWidget(row, 4, w_tol)
            self._pos_table.setCellWidget(row, 5, w_min)
            self._pos_table.setCellWidget(row, 6, w_max)
            self._pos_table.setCellWidget(row, 7, w_limits)
            self._pos_table.setCellWidget(row, 8, w_bp)

            btn = self._btn("Send")
            btn.clicked.connect(lambda _=False, j=joint: self._send_pos_row(j))
            self._pos_table.setCellWidget(row, 9, btn)

            self._pos_widgets[joint] = {
                "kp":         w_kp,
                "kd":         w_kd,
                "minpwm":     w_minpwm,
                "tol":        w_tol,
                "min":        w_min,
                "max":        w_max,
                "limits":     w_limits,
                "brakepoint": w_bp,
            }

        lay.addWidget(self._pos_table)

        # -- Quick action row per J1..J3 ------------------------------------
        lay.addWidget(self._hint_label(
            "Quick actions (per axis): Zero - zero, Goto - move to absolute "
            "angle, Rotate - move by relative angle, Stop - halt one axis."))
        self._pos_quick: dict[str, dict] = {}
        for joint in position_joints:
            r = QHBoxLayout()
            r.setSpacing(4)
            lbl = QLabel(self.ARM_JOINT_LABELS[joint])
            lbl.setMinimumWidth(110)
            r.addWidget(lbl)

            b_zero = self._btn("Zero")
            b_zero.clicked.connect(lambda _=False, j=joint: self._quick_zero(j))
            r.addWidget(b_zero)

            r.addWidget(QLabel("Goto:"))
            sp_goto = self._make_dspin(-360.0, 360.0, 0.0, 1)
            r.addWidget(sp_goto)
            b_goto = self._btn("Goto", primary=True)
            b_goto.clicked.connect(lambda _=False, j=joint, s=sp_goto:
                                   self._quick_goto(j, s.value()))
            r.addWidget(b_goto)

            r.addWidget(QLabel("Rotate:"))
            sp_rot = self._make_dspin(-360.0, 360.0, 0.0, 1)
            r.addWidget(sp_rot)
            b_rot = self._btn("Rotate")
            b_rot.clicked.connect(lambda _=False, j=joint, s=sp_rot:
                                  self._quick_rotate(j, s.value()))
            r.addWidget(b_rot)

            b_stop = self._btn("Stop", danger=True)
            b_stop.clicked.connect(lambda _=False, j=joint: self._quick_stop(j))
            r.addWidget(b_stop)

            r.addStretch()
            lay.addLayout(r)
            self._pos_quick[joint] = {"goto_spin": sp_goto, "rot_spin": sp_rot}
        return tab

    def _send_pos_row(self, joint: str):
        """Send ``set <j> ...`` position control fields."""
        w = self._pos_widgets.get(joint)
        if not w:
            return
        n = self.ARM_JOINT_NUM[joint]
        bp_txt = w["brakepoint"].text().strip()
        if bp_txt.upper() == self.BRAKEPOINT_OFF:
            bp_cmd = f"set {n} brakepoint off"
        else:
            try:
                bp_val = float(bp_txt)
                bp_cmd = f"set {n} brakepoint {bp_val:g}"
            except ValueError:
                bp_cmd = f"set {n} brakepoint off"
        cmds = [
            f"set {n} kp {w['kp'].value()}",
            f"set {n} kd {w['kd'].value()}",
            f"set {n} minpwm {w['minpwm'].value()}",
            f"set {n} tolerance {w['tol'].value():g}",
            f"set {n} minangle {w['min'].value():g}",
            f"set {n} maxangle {w['max'].value():g}",
            f"set {n} limits {1 if w['limits'].isChecked() else 0}",
            bp_cmd,
        ]
        self._gui.send_arm_setting_sequence(cmds)
        self._mark_dirty()

    def _quick_zero(self, joint: str):
        """Send ``zero <j>``; also clears target telemetry if tracked."""
        cmd = f"zero {joint}"
        self._gui.send_arm_setting_command(cmd)

    def _quick_goto(self, joint: str, angle: float):
        """Send ``goto <J> <angle>`` and update telemetry target if tracked."""
        if not self.ARM_HAS_POSITION[joint]:
            return
        cmd = f"goto {joint} {angle:g}"
        self._gui.send_arm_setting_command(cmd)

    def _quick_rotate(self, joint: str, angle: float):
        """Send ``rotate <J> <angle>`` and update telemetry target if tracked."""
        if not self.ARM_HAS_POSITION[joint]:
            return
        cmd = f"rotate {joint} {angle:g}"
        self._gui.send_arm_setting_command(cmd)

    def _quick_stop(self, joint: str):
        """Send ``stop <J>`` and clear target telemetry state."""
        cmd = f"stop {joint}"
        self._gui.send_arm_setting_command(cmd)

    # ======================================================================
    #  SECTION 3 - Sensor / AS5600 tab (J1..J3)
    # ======================================================================

    def _build_sensor_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(6)
        lay.setContentsMargins(8, 8, 8, 8)

        sensor_joints = ("J1", "J2", "J3")
        headers = ["Axis", "AS5600 CH", "Sensor", "Zero", "Read"]
        self._sensor_table = QTableWidget(len(sensor_joints), len(headers))
        self._sensor_table.setHorizontalHeaderLabels(headers)
        self._sensor_table.verticalHeader().setVisible(False)
        h = self._sensor_table.horizontalHeader()
        if h:
            h.setSectionResizeMode(0, QHeaderView.Stretch)
            h.setSectionResizeMode(1, QHeaderView.ResizeToContents)
            h.setSectionResizeMode(2, QHeaderView.Stretch)
            h.setSectionResizeMode(3, QHeaderView.ResizeToContents)
            h.setSectionResizeMode(4, QHeaderView.ResizeToContents)

        self._sensor_widgets: dict[str, dict] = {}
        for row, joint in enumerate(sensor_joints):
            self._sensor_table.setCellWidget(row, 0,
                                             self._static_label(self.ARM_JOINT_LABELS[joint]))
            w_ch = QComboBox()
            w_ch.addItems(self.AS5600_CHANNELS)
            w_ch.setCurrentIndex(0)
            w_ch.currentIndexChanged.connect(self._mark_dirty)
            self._sensor_table.setCellWidget(row, 1, w_ch)

            w_status = self._static_label("\u2014")
            self._sensor_table.setCellWidget(row, 2, w_status)

            b_zero = self._btn("Zero")
            b_zero.clicked.connect(lambda _=False, j=joint: self._quick_zero(j))
            self._sensor_table.setCellWidget(row, 3, b_zero)

            b_read = self._btn("Read")
            b_read.clicked.connect(lambda _=False, j=joint: self._gui.send_arm_setting_command("get sensors"))
            self._sensor_table.setCellWidget(row, 4, b_read)

            self._sensor_widgets[joint] = {"ch": w_ch, "status": w_status}

        lay.addWidget(self._sensor_table)

        # -- Send row for AS5600 channel assignment --------------------------
        r_send = QHBoxLayout()
        r_send.addWidget(QLabel("Set AS5600 channel for:"))
        self._sensor_as5600_joint = QComboBox()
        self._sensor_as5600_joint.addItems([self.ARM_JOINT_LABELS[j] for j in sensor_joints])
        r_send.addWidget(self._sensor_as5600_joint)
        b_apply = self._btn("Set AS5600", primary=True)
        b_apply.clicked.connect(self._send_as5600_channel)
        r_send.addWidget(b_apply)
        r_send.addStretch()
        lay.addLayout(r_send)

        # -- Global AS5600 buttons ------------------------------------------
        r_glb = QHBoxLayout()
        b_get = self._btn("Get Sensors")
        b_get.clicked.connect(lambda: self._gui.send_arm_setting_command("get sensors"))
        b_scan = self._btn("Scan AS5600")
        b_scan.clicked.connect(lambda: self._gui.send_arm_setting_command("get as5600"))
        b_off = self._btn("Stream Off", danger=True)
        b_off.clicked.connect(lambda: self._gui.send_arm_setting_command("stream off"))
        r_glb.addWidget(b_get)
        r_glb.addWidget(b_scan)
        r_glb.addWidget(b_off)
        r_glb.addStretch()
        lay.addLayout(r_glb)

        lay.addWidget(self._hint_label(
            "AS5600 channel uses ``set <j> as5600 <channel>`` (0..7) or "
            "``set <j> as5600 -1`` for OFF.  Continuous ``stream as5600`` is "
            "not enabled by default - the GUI already polls via the existing "
            "Manipulation Arm Telemetry polling."))
        return tab

    def _send_as5600_channel(self):
        label = self._sensor_as5600_joint.currentText()
        joint = next((j for j, l in self.ARM_JOINT_LABELS.items() if l == label),
                     None)
        if not joint or joint not in self._sensor_widgets:
            return
        n = self.ARM_JOINT_NUM[joint]
        ch_txt = self._sensor_widgets[joint]["ch"].currentText()
        if ch_txt == "OFF":
            self._gui.send_arm_setting_command(f"set {n} as5600 -1")
        else:
            self._gui.send_arm_setting_command(f"set {n} as5600 {ch_txt}")
        self._mark_dirty()

    def update_sensor_status(self, joint: str, text: str):
        """Called from the main GUI when sensor telemetry updates."""
        if joint in self._sensor_widgets:
            self._sensor_widgets[joint]["status"].setText(text)

    # ======================================================================
    #  SECTION 4 - Joystick / Keybind tab (J1..J6)
    # ======================================================================

    def _build_joystick_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(6)
        lay.setContentsMargins(8, 8, 8, 8)

        headers = ["Axis", "KeyFwd", "KeyBack",
                   "Test Fwd", "Test Back", "Stop"]
        self._keybind_table = QTableWidget(len(self.ARM_JOINTS), len(headers))
        self._keybind_table.setHorizontalHeaderLabels(headers)
        self._keybind_table.verticalHeader().setVisible(False)
        h = self._keybind_table.horizontalHeader()
        if h:
            h.setSectionResizeMode(0, QHeaderView.Stretch)
            for col in range(1, len(headers)):
                h.setSectionResizeMode(col, QHeaderView.ResizeToContents)

        self._keybind_widgets: dict[str, dict] = {}
        # Track toggle state for the Test buttons (per-joint). Each Test
        # button alternates ``joy button <id> 1`` / ``joy button <id> 0``.
        self._keybind_test_state: dict[str, dict[str, int]] = {
            j: {"fwd": 0, "back": 0} for j in self.ARM_JOINTS
        }
        for row, joint in enumerate(self.ARM_JOINTS):
            self._keybind_table.setCellWidget(row, 0,
                                              self._static_label(self.ARM_JOINT_LABELS[joint]))

            w_fwd = self._make_spin(-1, 32767, -1)
            w_back = self._make_spin(-1, 32767, -1)
            w_fwd.setSpecialValueText("-1 (none)")
            w_back.setSpecialValueText("-1 (none)")
            self._keybind_table.setCellWidget(row, 1, w_fwd)
            self._keybind_table.setCellWidget(row, 2, w_back)

            b_fwd = self._btn("Test Fwd")
            b_fwd.setCheckable(True)
            b_fwd.clicked.connect(lambda _=False, j=joint: self._test_key(j, "fwd"))
            self._keybind_table.setCellWidget(row, 3, b_fwd)

            b_back = self._btn("Test Back")
            b_back.setCheckable(True)
            b_back.clicked.connect(lambda _=False, j=joint: self._test_key(j, "back"))
            self._keybind_table.setCellWidget(row, 4, b_back)

            b_stop = self._btn("Stop", danger=True)
            b_stop.clicked.connect(lambda _=False, j=joint: self._quick_stop(j))
            self._keybind_table.setCellWidget(row, 5, b_stop)

            self._keybind_widgets[joint] = {"fwd": w_fwd, "back": w_back}

        lay.addWidget(self._keybind_table)

        # -- Set keybind row ------------------------------------------------
        r_set = QHBoxLayout()
        b_setkey = self._btn("Send All Keybinds", primary=True)
        b_setkey.clicked.connect(self._send_all_keybinds)
        b_refresh = self._btn("Refresh from params")
        b_refresh.clicked.connect(self._refresh_params)
        r_set.addWidget(b_setkey)
        r_set.addWidget(b_refresh)
        r_set.addStretch()
        lay.addLayout(r_set)

        lay.addWidget(self._hint_label(
            "Joystick commands only move the arm in ARM mode.  "
            "Send keybind: ``set <j> keybind <id>`` / ``set <j> negkeybind "
            "<id>`` (-1 disables).  Test buttons alternate "
            "``joy button <id> 1`` then ``joy button <id> 0``."))
        return tab

    def _send_all_keybinds(self):
        for joint in self.ARM_JOINTS:
            w = self._keybind_widgets[joint]
            n = self.ARM_JOINT_NUM[joint]
            fwd = w["fwd"].value()
            back = w["back"].value()
            self._gui.send_arm_setting_command(f"set {n} keybind {fwd}")
            self._gui.send_arm_setting_command(f"set {n} negkeybind {back}")
        self._mark_dirty()

    def _test_key(self, joint: str, which: str):
        """Toggle the ``joy button`` test for the joint."""
        w = self._keybind_widgets.get(joint)
        if not w:
            return
        btn_id = w[which].value()
        if btn_id < 0:
            self.set_status(
                f"Cannot test {joint} {which}: KeyFwd/KeyBack is -1 (none).",
                "warn")
            return
        state = self._keybind_test_state[joint][which]
        new_state = 0 if state == 1 else 1
        self._keybind_test_state[joint][which] = new_state
        self._gui.send_arm_setting_command(f"joy button {btn_id} {new_state}")

    # ======================================================================
    #  SECTION 5 - System / Save tab
    # ======================================================================

    def _build_system_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(8)
        lay.setContentsMargins(8, 8, 8, 8)

        # -- Mode & safety block --------------------------------------------
        grp_mode = QGroupBox("Mode & Stop")
        gm = QVBoxLayout(grp_mode)
        gm.setSpacing(6)
        gm.setContentsMargins(10, 14, 10, 10)
        r1 = QHBoxLayout()
        b_arm = self._btn("Mode ARM Confirm", primary=True)
        b_arm.clicked.connect(lambda: self._gui.send_arm_setting_command("mode arm confirm"))
        b_safe = self._btn("Mode SAFE", danger=True)
        b_safe.clicked.connect(lambda: self._gui.send_arm_setting_command("mode safe"))
        r1.addWidget(b_arm)
        r1.addWidget(b_safe)
        r1.addStretch()
        gm.addLayout(r1)
        r2 = QHBoxLayout()
        b_stop = self._btn("Stop", danger=True)
        b_stop.clicked.connect(lambda: self._gui.send_arm_setting_command("stop"))
        b_stopall = self._btn("Stop All", danger=True)
        b_stopall.clicked.connect(lambda: self._gui.send_arm_setting_command("stopall"))
        b_hb = self._btn("Heartbeat")
        b_hb.clicked.connect(lambda: self._gui.send_arm_setting_command("heartbeat"))
        r2.addWidget(b_stop)
        r2.addWidget(b_stopall)
        r2.addWidget(b_hb)
        r2.addStretch()
        gm.addLayout(r2)
        lay.addWidget(grp_mode)

        # -- Diagnostic block ------------------------------------------------
        grp_diag = QGroupBox("Diagnostics")
        gd = QVBoxLayout(grp_diag)
        gd.setSpacing(6)
        gd.setContentsMargins(10, 14, 10, 10)
        r3 = QHBoxLayout()
        b_mode = self._btn("Get Mode")
        b_mode.clicked.connect(lambda: self._gui.send_arm_setting_command("get mode"))
        b_fault = self._btn("Get Fault")
        b_fault.clicked.connect(lambda: self._gui.send_arm_setting_command("get fault"))
        b_params = self._btn("Refresh Params")
        b_params.clicked.connect(self._refresh_params)
        r3.addWidget(b_mode)
        r3.addWidget(b_fault)
        r3.addWidget(b_params)
        r3.addStretch()
        gd.addLayout(r3)
        lay.addWidget(grp_diag)

        # -- Save block -----------------------------------------------------
        grp_save = QGroupBox("Save")
        gs = QVBoxLayout(grp_save)
        gs.setSpacing(6)
        gs.setContentsMargins(10, 14, 10, 10)
        b_save = self._btn("Save Settings", primary=True)
        b_save.clicked.connect(self._save_settings)
        gs.addWidget(b_save)
        gs.addWidget(self._hint_label(
            "Save requires SAFE mode and all motors stopped.  Clicking Save "
            "sends: stopall, then mode safe, then save."))
        lay.addWidget(grp_save)

        # -- Heartbeat timeout block ----------------------------------------
        grp_hb = QGroupBox("Heartbeat Timeout")
        gh = QVBoxLayout(grp_hb)
        gh.setSpacing(6)
        gh.setContentsMargins(10, 14, 10, 10)
        rh = QHBoxLayout()
        rh.addWidget(QLabel("Timeout (ms):"))
        self._spin_heartbeat = self._make_spin(0, 60000, 1000)
        rh.addWidget(self._spin_heartbeat)
        b_hb_set = self._btn("Send", primary=True)
        b_hb_set.clicked.connect(self._send_heartbeat)
        rh.addWidget(b_hb_set)
        rh.addStretch()
        gh.addLayout(rh)
        lay.addWidget(grp_hb)

        lay.addStretch()
        return tab

    def _send_heartbeat(self):
        self._gui.send_arm_setting_command(
            f"set heartbeat {self._spin_heartbeat.value()}")
        self._mark_dirty()

    def _save_settings(self):
        """Confirm and send the safe save sequence.

        Sends ``stopall`` -> ``mode safe`` -> ``save`` with a small delay so
        the F401 firmware has time to honour each precondition.  The dialog
        status hint reflects save success / failure once the F401 replies.
        """
        c = self._gui._colors()
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Question)
        msg.setWindowTitle("Save Settings")
        msg.setText("Save settings to F401 flash?")
        msg.setInformativeText(
            "Saving requires SAFE mode and all motors stopped. "
            "The GUI will send:\n"
            "  1. stopall\n"
            "  2. mode safe\n"
            "  3. save\n\n"
            "Continue?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        msg.setStyleSheet(self.styleSheet())
        if msg.exec() != QMessageBox.Yes:
            self.set_status("Save cancelled by user.", None)
            return
        self.set_status("Saving: stopping all motors...", "warn")
        self._gui.send_arm_setting_sequence(
            ["stopall", "mode safe", "save"], interval_ms=250)

    # ======================================================================
    #  Bottom bar
    # ======================================================================

    def _build_bottom_bar(self) -> QHBoxLayout:
        bar = QHBoxLayout()
        bar.setSpacing(8)
        b_refresh = self._btn("Refresh Params")
        b_refresh.clicked.connect(self._refresh_params)
        b_save = self._btn("Save Settings", primary=True)
        b_save.clicked.connect(self._save_settings)
        b_close = self._btn("Close")
        b_close.clicked.connect(self.close)
        bar.addWidget(b_refresh)
        bar.addWidget(b_save)
        bar.addStretch()
        bar.addWidget(b_close)
        return bar

    # ======================================================================
    #  Public API used by the main GUI (params / save result / dirty hints)
    # ======================================================================

    def apply_params(self, params: dict[str, dict]):
        """Populate dialog fields from a parsed params dict.

        ``params`` is keyed by joint ("J1".."J6") and each value is a dict of
        the parsed KEY=VALUE pairs (strings).  Unknown / missing keys are
        ignored silently so a partial or extended F401 output cannot crash
        the dialog.  Per-joint dicts that are entirely missing are skipped.
        """
        for joint in self.ARM_JOINTS:
            jpr = params.get(joint)
            if not jpr:
                continue
            # Axis table
            aw = self._axis_widgets.get(joint)
            if aw:
                if "INVERT" in jpr:
                    try:
                        aw["invert"].setChecked(int(jpr["INVERT"]) != 0)
                    except (TypeError, ValueError):
                        pass
                if "MAXPWM" in jpr:
                    self._safe_set_spin(aw["maxpwm"], jpr["MAXPWM"])
                if "DEFAULT" in jpr:
                    self._safe_set_spin(aw["default"], jpr["DEFAULT"])
                if "POSGAIN" in jpr:
                    self._safe_set_spin(aw["posgain"], jpr["POSGAIN"])
                if "NEGGAIN" in jpr:
                    self._safe_set_spin(aw["neggain"], jpr["NEGGAIN"])
                if "DEAD" in jpr:
                    self._safe_set_spin(aw["dead"], jpr["DEAD"])
                if "STOPMODE" in jpr:
                    idx = aw["stopmode"].findText(jpr["STOPMODE"].lower(),
                                                  Qt.MatchFixedString)
                    if idx >= 0:
                        aw["stopmode"].setCurrentIndex(idx)
            # Position table - J1..J3
            pw = self._pos_widgets.get(joint)
            if pw:
                if "KP" in jpr:
                    self._safe_set_spin(pw["kp"], jpr["KP"])
                if "KD" in jpr:
                    self._safe_set_spin(pw["kd"], jpr["KD"])
                if "MINPWM" in jpr:
                    self._safe_set_spin(pw["minpwm"], jpr["MINPWM"])
                if "TOL" in jpr:
                    try:
                        pw["tol"].setValue(float(jpr["TOL"]))
                    except (TypeError, ValueError):
                        pass
                if "MIN" in jpr:
                    try:
                        pw["min"].setValue(float(jpr["MIN"]))
                    except (TypeError, ValueError):
                        pass
                if "MAX" in jpr:
                    try:
                        pw["max"].setValue(float(jpr["MAX"]))
                    except (TypeError, ValueError):
                        pass
                if "LIMITS" in jpr:
                    try:
                        pw["limits"].setChecked(int(jpr["LIMITS"]) != 0)
                    except (TypeError, ValueError):
                        pass
                if "BRAKEPOINT" in jpr:
                    val = jpr["BRAKEPOINT"]
                    if isinstance(val, str) and val.upper() in ("OFF", "-1"):
                        pw["brakepoint"].setText(self.BRAKEPOINT_OFF)
                    else:
                        try:
                            pw["brakepoint"].setText(f"{float(val):g}")
                        except (TypeError, ValueError):
                            pw["brakepoint"].setText(str(val))
            # AS5600 table - J1..J3
            sw = self._sensor_widgets.get(joint)
            if sw and "AS5600" in jpr:
                val = jpr["AS5600"]
                try:
                    ch = int(val)
                    txt = "OFF" if ch < 0 else str(ch)
                except (TypeError, ValueError):
                    txt = "OFF"
                idx = sw["ch"].findText(txt)
                if idx >= 0:
                    sw["ch"].setCurrentIndex(idx)
            # Keybind table - J1..J6
            kw = self._keybind_widgets.get(joint)
            if kw:
                if "KEYFWD" in jpr:
                    self._safe_set_spin(kw["fwd"], jpr["KEYFWD"])
                if "KEYBACK" in jpr:
                    self._safe_set_spin(kw["back"], jpr["KEYBACK"])
        # Newly refreshed from params: not considered dirty.
        self._dirty = False
        self.set_status("Refreshed from params.", None)

    def notify_save_success(self):
        """Called when the F401 reports ``OK SAVED_FLASH``."""
        self.clear_dirty()

    def notify_save_failure(self, err_text: str):
        """Called when the F401 reports a save error."""
        c = self._gui._colors()
        self._lbl_status.setStyleSheet(
            f"color: {c['danger_bright']}; font-size: 11px; border: none;")
        self.set_status(f"Save failed: {err_text}", "err")

    def notify_settings_dirty(self):
        """Called when the F401 fault report indicates SETTINGS_DIRTY=1."""
        c = self._gui._colors()
        self._lbl_status.setStyleSheet(
            f"color: {c['warning']}; font-size: 11px; border: none;")
        self.set_status("\u26A0 RAM settings not saved (SETTINGS_DIRTY=1).",
                        "warn")

    # ======================================================================
    #  Small widget factory helpers
    # ======================================================================

    def _static_label(self, text: str) -> QLabel:
        l = QLabel(text)
        l.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        return l

    def _make_spin(self, mn: int, mx: int, val: int) -> QSpinBox:
        s = QSpinBox()
        s.setRange(mn, mx)
        s.setValue(val)
        s.setFixedWidth(80)
        s.setAlignment(Qt.AlignCenter)
        s.valueChanged.connect(self._mark_dirty)
        return s

    def _make_dspin(self, mn: float, mx: float, val: float,
                    decimals: int = 1) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(mn, mx)
        s.setDecimals(decimals)
        s.setValue(val)
        s.setFixedWidth(80)
        s.setAlignment(Qt.AlignCenter)
        s.setSingleStep(1.0)
        s.valueChanged.connect(self._mark_dirty)
        return s

    @staticmethod
    def _safe_set_spin(spin, value):
        """Set a QSpinBox/QDoubleSpinBox from a string without raising.

        Silently ignores invalid values so a malformed params line cannot
        crash the dialog.
        """
        try:
            if isinstance(spin, QDoubleSpinBox):
                spin.setValue(float(value))
            else:
                spin.setValue(int(float(value)))
        except (TypeError, ValueError):
            pass

    def closeEvent(self, event):
        """Do not send any movement / stop command on close."""
        event.accept()


# ============================================================================
#  Main GUI
# ============================================================================

class EarendilControlGui(QMainWindow):
    """
    Main rover control window.
    Left side: control panels.  Right side: GUI console.
    Connects to the Raspberry Pi TCP-to-Serial bridge.
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
            background: transparent;
        }}
        QWidget#sidePanel {{
            background: transparent;
        }}
        QScrollArea, QScrollArea > QWidget, QScrollArea > QWidget > QWidget {{
            background: transparent;
        }}
        QSplitter {{
            background: transparent;
        }}
        QGroupBox {{
            background-color: {c['bg_panel']};
            border: 1px solid {c['border']};
            border-radius: 6px;
            margin-top: 6px;
            margin-bottom: 0px;
            padding-top: 10px;
            padding-bottom: 0px;
            font-weight: bold;
            color: {c['text']};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
            color: {c['accent_gold']};
        }}
        QPushButton {{
            background-color: {c['bg_input']};
            border: 1px solid {c['border']};
            border-radius: 6px;
            padding: 4px 10px;
            min-height: 22px;
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
    ARC_REPEAT_INTERVAL_MS = 500
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

    # TCP teardown reasons.  The socket state remains the transport source of
    # truth; these values record why an active attempt/session is ending so
    # asynchronous Qt signals cannot change the operator-facing outcome.
    TCP_USER_DISCONNECT = "USER_DISCONNECT"
    TCP_USER_CANCEL = "USER_CANCEL"
    TCP_CONNECT_TIMEOUT = "CONNECT_TIMEOUT"
    TCP_SOCKET_ERROR = "SOCKET_ERROR"
    TCP_REMOTE_CLOSE = "REMOTE_CLOSE"
    TCP_RX_OVERFLOW = "RX_OVERFLOW"
    TCP_TX_OVERFLOW = "TX_OVERFLOW"
    TCP_WINDOW_CLOSE = "WINDOW_CLOSE"
    TCP_STALE_CONNECT = "STALE_CONNECT"

    # Authoritative GUI-side state for the H7's shared MPU/MAG periodic
    # stream.  The H7 emits an IMU_STREAM confirmation for explicit changes;
    # individual one-shot sensor replies do not change this state.
    IMU_STREAM_UNKNOWN = "UNKNOWN"
    IMU_STREAM_STARTING = "STARTING"
    IMU_STREAM_ON = "ON"
    IMU_STREAM_STOPPING = "STOPPING"
    IMU_STREAM_OFF = "OFF"

    # -- F411 Motor Fault Code Reference -----------------------------------
    FAULT_CODES = [
        (0,  "NONE",         "No fault. System is operating normally.",                           "No action required."),
        (1,  "NO_HALL",      "Hall sensor feedback is missing or lost.",                          "Check Hall sensor cable, Hall power, Hall ground, sensor connector, and motor wiring."),
        (2,  "INVALID_HALL", "Invalid Hall sensor pattern detected.",                             "BLDC Hall state should be one of the valid six states. Patterns like 000 or 111 indicate wiring, sensor, or signal problems."),
        (3,  "ILLEGAL_TRANS","Illegal Hall sensor transition detected.",                          "Hall state sequence jumped, reversed, or changed unexpectedly. Check Hall wire order, phase wire order, and motor direction matching."),
        (4,  "HOST_LOST",    "Communication with the host controller was lost.",                   "The F411 motor driver stopped receiving data from H7. Check UART link, cabling, common ground, and command timing."),
        (5,  "WATCHDOG",     "Command watchdog timeout.",                                          "No fresh drive command was received within the expected time while the motor was active. Check command repeat, GUI TX, H7 forwarding, and UART timing."),
        (6,  "HW_BREAK",     "Hardware break protection fault.",                                   "Reserved for timer break, gate-driver emergency shutdown, or hardware-level protection. May not be active on current hardware."),
        (7,  "ESTOP",        "Emergency stop fault.",                                              "estop command or emergency stop path was triggered. Motor output is shut down for safety."),
        (8,  "OVERCURRENT",  "Overcurrent fault.",                                                 "Motor or driver current exceeded the safe limit. Check motor load, wiring, MOSFET stage, and current sensing. May not be active if current measurement is not implemented."),
        (9,  "OVERVOLTAGE",  "Overvoltage fault.",                                                 "Supply voltage exceeded the safe limit. Check battery voltage, regeneration effects, and power input. May not be active in current firmware."),
        (10, "UNDERVOLTAGE", "Undervoltage fault.",                                                "Supply voltage dropped below the safe limit. Check battery level, cable resistance, connectors, and voltage sag under load. May not be active in current firmware."),
        (11, "OVERTEMP",     "Overtemperature fault.",                                              "Motor driver, MOSFETs, or PCB temperature is too high. Check cooling, load, current draw, and temperature sensor support. May not be active in current firmware."),
        (12, "GATE_DRIVER",  "Gate driver fault.",                                                  "Gate driver reported a fault such as UVLO, drive failure, or protection event. Check L6388ED/gate driver supply, bootstrap, driver signals, and MOSFET stage."),
        (13, "UART_RX_OVF",  "UART RX buffer overflow.",                                           "Incoming serial data overflowed the receive buffer. Possible causes: too frequent commands, malformed packets, parser overload, or communication congestion."),
    ]

    def _fault_name(self, code_str: str) -> str:
        """Return the short name for a fault code string, or 'UNKNOWN'."""
        try:
            idx = int(code_str)
            if 0 <= idx < len(self.FAULT_CODES):
                return self.FAULT_CODES[idx][1]
        except (ValueError, IndexError):
            pass
        return "UNKNOWN"

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
        Qt.Key_T: "T",
        Qt.Key_Y: "Y",
        Qt.Key_G: "G",
        Qt.Key_H: "H",
    }

    # -- Manipulation Arm Telemetry ----------------------------------------
    ARM_JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
    ARM_JOINT_LABELS = {
        "J1": "J1 Base",
        "J2": "J2 Shoulder",
        "J3": "J3 Elbow",
        "J4": "J4 Wrist Pitch",
        "J5": "J5 Wrist Twist",
        "J6": "J6 Gripper",
    }
    ARM_MOTOR_MAP = {"J1": "M1", "J2": "M2", "J3": "M3", "J6": "M6"}
    ARM_HAS_SENSOR = {"J1": True, "J2": True, "J3": True, "J4": False, "J5": False, "J6": False}
    ARM_HAS_TARGET = {"J1": True, "J2": True, "J3": True, "J4": False, "J5": False, "J6": False}
    ARM_HAS_LIMIT = {"J1": True, "J2": True, "J3": True, "J4": False, "J5": False, "J6": False}
    ARM_COL_HEADERS = ["Axis", "Degree", "Tgt", "Dir", "PWM", "Brake", "Stop", "Limit", "Sens", "Fault"]
    ARM_COL = {name: i for i, name in enumerate(ARM_COL_HEADERS)}
    ARM_POLL_MOTORS_MS = 250
    ARM_POLL_SENSORS_MS = 500
    ARM_POLL_FAULT_MS = 1000

    # -- Drill Telemetry (F401 M4/M5/M6 in DRILL mode) --------------------
    DRILL_PARTS = ("elevator_l", "elevator_r", "drill")
    DRILL_PART_LABELS = {
        "elevator_l": "Elevator L",
        "elevator_r": "Elevator R",
        "drill":       "Drill Motor",
    }
    # In DRILL mode M4=elevator-left, M5=elevator-right, M6=drill motor.
    DRILL_MOTOR_MAP = {"elevator_l": "M4", "elevator_r": "M5", "drill": "M6"}
    DRILL_COL_HEADERS = ["Part", "Dir", "PWM", "Brake", "EN", "State", "Fault"]
    DRILL_COL = {name: i for i, name in enumerate(DRILL_COL_HEADERS)}
    DRILL_POLL_MOTORS_MS = 400
    DRILL_POLL_FAULT_MS = 1000
    DRILL_POLL_MODE_MS = 1000

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Earendil - Rover Control")
        self.setMinimumSize(1100, 650)

        # -- State ----------------------------------------------------------
        self.current_theme = "light"            # "dark" or "light"

        # -- TCP connection state -------------------------------------------
        self._tcp_socket: QTcpSocket | None = None
        self._tcp_rx_buffer = bytearray()
        self._tcp_connect_timer = QTimer(self)
        self._tcp_connect_timer.setSingleShot(True)
        self._tcp_connect_timer.setInterval(DEFAULT_CONNECT_TIMEOUT_MS)
        self._tcp_connect_timer.timeout.connect(self._on_tcp_connect_timeout)
        self.connected = False  # True when TCP is connected (synced from socket state)
        self._tcp_session_id: int = 0  # monotonically increasing; incremented on each new connection attempt
        self._tcp_attempt_session_id: int | None = None
        self._tcp_connected_session_id: int | None = None
        self._tcp_teardown_session_id: int | None = None
        self._tcp_finalized_session_id: int | None = None
        self._tcp_prepared_session_id: int | None = None
        self._tcp_teardown_reason: str | None = None
        self._tcp_teardown_detail: str | None = None
        self._window_closing: bool = False
        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.setInterval(CONTROL_HEARTBEAT_PERIOD_MS)
        self._heartbeat_timer.timeout.connect(self._send_heartbeat)
        self._linkstat_retry_timers: list[QTimer] = []
        for delay_ms in LINKSTAT_RETRY_DELAYS_MS:
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.setInterval(delay_ms)
            timer.setProperty("tcp_linkstat_connected", False)
            self._linkstat_retry_timers.append(timer)

        # -- H7 control-link status -----------------------------------------
        self._h7_link_status = "UNKNOWN"   # "UNKNOWN" / "ALIVE" / "TIMEOUT"

        self.mode = "RPM"               # "RPM" or "DUTY"
        self.fb_rpm = self.DEFAULT_RPM_FB
        self.rot_rpm = self.DEFAULT_RPM_ROT
        self.fb_pwm = self.DEFAULT_PWM_FB
        self.rot_pwm = self.DEFAULT_PWM_ROT
        self.turn_ratio = 0.50

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

        # -- cfgcache read state ----------------------------------------
        self._cfgread_motor: str | None = None          # motor being read
        self._cfgread_dialog = None                      # MotorSettingsDialog ref
        self._cfgread_pending: dict = {}                 # accumulated cfg lines
        self._cfgread_retry_count: int = 0
        self._cfgread_retry_timer = QTimer(self)
        self._cfgread_retry_timer.setInterval(400)
        self._cfgread_retry_timer.timeout.connect(self._cfgread_retry_fetch)
        self._cfgread_timeout_timer = QTimer(self)
        self._cfgread_timeout_timer.setSingleShot(True)
        self._cfgread_timeout_timer.setInterval(2500)
        self._cfgread_timeout_timer.timeout.connect(self._cfgread_on_timeout)
        self._cfgread_apply_timer = QTimer(self)
        self._cfgread_apply_timer.setSingleShot(True)
        self._cfgread_apply_timer.setInterval(150)
        self._cfgread_apply_timer.timeout.connect(self._cfgread_apply_now)

        # -- Persistent cfg cache (survives dialog close/reopen) --------
        self._last_f411_cfg_by_motor: dict[str, dict] = {}
        self._last_f411_cfg_motor: str | None = None

        # -- IMU/MAG settings dialog reference ---------------------------
        self._imu_settings_dialog: ImuMagSettingsDialog | None = None

        # -- Manipulation Arm Settings dialog reference ------------------
        self._arm_settings_dialog: ManipulationArmSettingsDialog | None = None
        # Latest parsed F401 params (per joint) - kept here so it survives
        # dialog close/reopen and is applied when the dialog reappears.
        self._arm_params_cache: dict[str, dict] = {}

        # -- Manipulation Arm Telemetry state ---------------------------
        self._arm_state: dict[str, dict[str, str]] = {
            j: {
                "degree": "—", "tgt": "—", "dir": "STOP", "pwm": "0",
                "brake": "OFF", "stop": "COAST", "limit": "OK" if self.ARM_HAS_LIMIT[j] else "—",
                "sens": "—", "fault": "OK",
            } for j in self.ARM_JOINTS
        }
        self._arm_tgt_angle: dict[str, float | None] = {j: None for j in self.ARM_JOINTS}
        self._arm_sensors_seen: dict[str, bool] = {j: False for j in self.ARM_JOINTS}
        self._arm_heartbeat_active: bool = False

        # -- Drill Telemetry state (F401 M4/M5/M6 in DRILL mode) --------
        # Compact state model for the Drill Telemetry panel.  Mirrors the
        # Manipulation Arm Telemetry style: defaults are shown until
        # telemetry arrives.  ``mode`` is updated by the F401 ``get mode``
        # parser and the mode-switch buttons via RX confirmation lines.
        self._drill_state: dict = {
            "mode": "UNKNOWN",
            "parts": {
                "elevator_l": {"dir": "STOP", "pwm": "0", "brake": "OFF",
                                "en": "OFF", "state": "IDLE", "fault": "OK"},
                "elevator_r": {"dir": "STOP", "pwm": "0", "brake": "OFF",
                                "en": "OFF", "state": "IDLE", "fault": "OK"},
                "drill":      {"dir": "STOP", "pwm": "0", "brake": "OFF",
                                "en": "OFF", "state": "IDLE", "fault": "OK"},
            },
            "activity": "UNKNOWN",
            "last_commanded_activity": "UNKNOWN",
            "heartbeat_fault": False,
        }

        # -- Telemetry freshness tracking --------------------------------
        # Timestamps (time.monotonic) of last valid telemetry per subsystem.
        # 0.0 means "never received since connect".
        self._freshness_motor: dict[str, float] = {m: 0.0 for m in ("FL", "FR", "RL", "RR")}
        self._freshness_imu: float = 0.0
        self._freshness_mag: float = 0.0
        self._freshness_arm: float = 0.0
        self._freshness_drill: float = 0.0
        # Previous stale states for transition logging.
        self._freshness_motor_stale: dict[str, bool] = {m: False for m in ("FL", "FR", "RL", "RR")}
        self._freshness_imu_stale: bool = False
        self._freshness_mag_stale: bool = False
        self._freshness_arm_stale: bool = False
        self._freshness_drill_stale: bool = False
        self._imu_stream_state: str = self.IMU_STREAM_UNKNOWN
        self._imu_one_shot_session_id: int | None = None
        self._imu_one_shot_deadline: float = 0.0
        self._mag_one_shot_session_id: int | None = None
        self._mag_one_shot_deadline: float = 0.0
        self._imu_stream_detect_session_id: int | None = None
        self._imu_stream_detect_first_rx: float = 0.0
        self._imu_stream_detect_count: int = 0
        self._imu_stream_detect_suppress_session_id: int | None = None
        self._imu_stream_detect_suppress_until: float = 0.0
        # Telemetry expectation: True = telemetry actively expected,
        # False = intentionally idle (not expected), None = not yet determined.
        self._telemetry_expected: dict[str, bool | None] = {
            "FL": None, "FR": None, "RL": None, "RR": None,
            "IMU": None, "MAG": None, "ARM": False, "DRILL": False,
        }
        # Monotonic timestamp when expectation became True for a subsystem.
        # Used to measure age from expectation start when no telemetry has
        # been received yet (last_rx == 0.0).
        self._telemetry_expected_since: dict[str, float] = {
            "FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0,
            "IMU": 0.0, "MAG": 0.0, "ARM": 0.0, "DRILL": 0.0,
        }
        # Real motor link state (LOST/OK) preserved separately from freshness.
        self._motor_link_state: dict[str, str] = {m: "UNKNOWN" for m in ("FL", "FR", "RL", "RR")}
        self._freshness_timer = QTimer(self)
        self._freshness_timer.setInterval(TELEMETRY_FRESHNESS_CHECK_MS)
        self._freshness_timer.timeout.connect(self._check_telemetry_freshness)

        # -- Build UI -------------------------------------------------------
        self._central = LogoBackgroundWidget("earendil_logo.png", opacity=1.0)
        central = self._central
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel (wrapped in QScrollArea for overflow)
        left_panel = QWidget()
        left_panel.setObjectName("sidePanel")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        # -- Top row: Network Connection (left) + Rover Status (right) ---
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(4)
        top_row.addWidget(self._build_connection_group())
        top_row.addWidget(self._build_rover_status_group(), 1)

        left_layout.addLayout(top_row)

        # -- Mode / Value  (left)  +  Operating Mode (right)  in one row --
        mode_op_row = QHBoxLayout()
        mode_op_row.setContentsMargins(0, 0, 0, 0)
        mode_op_row.setSpacing(4)
        mode_op_row.addWidget(self._build_mode_value_group(), 1)
        mode_op_row.addWidget(self._build_operating_mode_group())
        left_layout.addLayout(mode_op_row)

        left_layout.addWidget(self._build_mobility_mode_group())

        left_layout.addWidget(self._build_motor_table_group())

        # Manipulation Arm Telemetry + Drill Telemetry side-by-side in a
        # QSplitter so the user can drag the divider.  Arm gets 3/5 of
        # the available width, drill gets 2/5; minimum widths ensure both
        # tables stay readable.
        arm_drill_splitter = QSplitter(Qt.Horizontal)
        arm_grp = self._build_arm_telemetry_group()
        drill_grp = self._build_drill_telemetry_group()
        arm_drill_splitter.addWidget(arm_grp)
        arm_drill_splitter.addWidget(drill_grp)
        arm_drill_splitter.setStretchFactor(0, 3)
        arm_drill_splitter.setStretchFactor(1, 2)
        arm_grp.setMinimumWidth(620)
        drill_grp.setMinimumWidth(450)
        left_layout.addWidget(arm_drill_splitter)

        imu_env_row = QHBoxLayout()
        imu_env_row.setContentsMargins(0, 0, 0, 0)
        imu_env_row.setSpacing(8)
        imu_env_row.addWidget(self._build_imu_group())
        imu_env_row.addStretch()
        left_layout.addLayout(imu_env_row)

        # Wrap left_panel in a QScrollArea so content is accessible
        # when the window is shorter than the combined widget heights.
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_panel)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.viewport().setAutoFillBackground(False)
        left_scroll.setStyleSheet("QScrollArea { background: transparent; }")
        splitter.addWidget(left_scroll)

        # Right panel - console
        splitter.addWidget(self._build_console_group())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # -- Repeat timer ---------------------------------------------------
        self._repeat_timer = QTimer(self)
        self._repeat_timer.setInterval(self.REPEAT_INTERVAL_MS)
        self._repeat_timer.timeout.connect(self._repeat_movement)

        # -- Arc-turn repeat timer (T/Y) ----------------------------------
        self._arc_repeat_timer = QTimer(self)
        self._arc_repeat_timer.setInterval(self.ARC_REPEAT_INTERVAL_MS)
        self._arc_repeat_timer.timeout.connect(self._repeat_arc_turn)

        # -- Operating-mode confirmation timeout ---------------------------
        # Started when a mode button sends a command; if the H7 does not
        # reply with a `[MODE] ... active` line in time, we warn and keep
        # the previously confirmed mode (no optimistic UI change).
        self._pending_mode_timer = QTimer(self)
        self._pending_mode_timer.setSingleShot(True)
        self._pending_mode_timer.setInterval(self.OP_MODE_CONFIRM_TIMEOUT_MS)
        self._pending_mode_timer.timeout.connect(self._on_pending_mode_timeout)

        # -- Arm telemetry polling timers ----------------------------------
        self._arm_poll_motors_timer = QTimer(self)
        self._arm_poll_motors_timer.setInterval(self.ARM_POLL_MOTORS_MS)
        self._arm_poll_motors_timer.timeout.connect(self._arm_poll_motors)

        self._arm_poll_sensors_timer = QTimer(self)
        self._arm_poll_sensors_timer.setInterval(self.ARM_POLL_SENSORS_MS)
        self._arm_poll_sensors_timer.timeout.connect(self._arm_poll_sensors)

        self._arm_poll_fault_timer = QTimer(self)
        self._arm_poll_fault_timer.setInterval(self.ARM_POLL_FAULT_MS)
        self._arm_poll_fault_timer.timeout.connect(self._arm_poll_fault)

        # -- Drill telemetry polling timers --------------------------------
        # Lightweight polling only fires while the TCP connection is active
        # (see _drill_poll_*).  A single ``get motors`` request feeds both
        # the Manipulation Arm Telemetry and the Drill Telemetry tables.
        self._drill_poll_motors_timer = QTimer(self)
        self._drill_poll_motors_timer.setInterval(self.DRILL_POLL_MOTORS_MS)
        self._drill_poll_motors_timer.timeout.connect(self._drill_poll_motors)

        self._drill_poll_fault_timer = QTimer(self)
        self._drill_poll_fault_timer.setInterval(self.DRILL_POLL_FAULT_MS)
        self._drill_poll_fault_timer.timeout.connect(self._drill_poll_fault)

        self._drill_poll_mode_timer = QTimer(self)
        self._drill_poll_mode_timer.setInterval(self.DRILL_POLL_MODE_MS)
        self._drill_poll_mode_timer.timeout.connect(self._drill_poll_mode)

        # Prevent buttons from stealing keyboard focus (Space must always reach keyPressEvent)
        for btn in self.findChildren(QPushButton):
            btn.setFocusPolicy(Qt.NoFocus)

        self.setFocusPolicy(Qt.StrongFocus)

        # Apply the active theme to all widgets (stylesheet + inline styles +
        # background logo).  Done last so all widgets exist before re-styling.
        self._apply_theme()

        self._log_info("Ready. Connect to rover to begin.")

    # ======================================================================
    #  UI Builders
    # ======================================================================

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("Network Connection")
        grp.setMaximumWidth(800)
        grp.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(3)

        lay.addWidget(QLabel("Raspberry Pi IP:"))
        self._host_edit = QLineEdit(DEFAULT_BRIDGE_HOST)
        self._host_edit.setFixedWidth(140)
        lay.addWidget(self._host_edit)

        lay.addWidget(QLabel("TCP Port:"))
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        self._port_spin.setValue(DEFAULT_BRIDGE_PORT)
        self._port_spin.setFixedWidth(80)
        lay.addWidget(self._port_spin)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setFixedWidth(105)
        self._btn_connect.setStyleSheet("QPushButton { background-color: #1e6e3e; color: #C0C0C0; }")
        self._btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self._btn_connect)

        self._lbl_status = QLabel("Status: DISCONNECTED")
        self._lbl_status.setStyleSheet("color: #B00020; font-weight: bold;")
        self._lbl_status.setFixedWidth(170)
        lay.addWidget(self._lbl_status)

        self._init_tcp_socket()
        return grp

    def _build_rover_status_group(self) -> QGroupBox:
        grp = QGroupBox("Rover Status")
        grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(6, 2, 6, 2)
        lay.setSpacing(10)

        c = self._colors()

        def _badge(initial: str, color: str) -> QLabel:
            lbl = QLabel(initial)
            lbl.setStyleSheet(self._style_badge(color))
            return lbl

        self._lbl_qs_mode = _badge("Mode: RPM", c['accent_gold'])
        lay.addWidget(self._lbl_qs_mode)

        self._lbl_qs_motion = _badge("Motion: IDLE", c['text_muted'])
        lay.addWidget(self._lbl_qs_motion)

        self._lbl_qs_port = _badge("Link: Disconnected", c['danger'])
        lay.addWidget(self._lbl_qs_port)

        self._lbl_qs_h7_link = _badge("H7 Control Link: UNKNOWN", c['text_muted'])
        lay.addWidget(self._lbl_qs_h7_link)

        lay.addStretch()

        # Theme toggle button - only changes visual theme, never sends a
        # network command.  Text reflects the theme we will switch TO.
        self._btn_theme = QPushButton("Dark Mode")
        self._btn_theme.setFixedWidth(100)
        self._btn_theme.clicked.connect(self._toggle_theme)
        lay.addWidget(self._btn_theme)
        return grp

    def _build_mode_value_group(self) -> QGroupBox:
        grp = QGroupBox("Mode / Value")
        grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(6, 2, 6, 2)
        lay.setSpacing(6)

        lay.addWidget(QLabel("Mode:"))
        self._lbl_mode = QLabel("RPM")
        self._lbl_mode.setStyleSheet(
            "color: #D4AF37; font-size: 13px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_mode)

        lay.addSpacing(10)
        self._lbl_fb_label = QLabel("FB RPM:")
        lay.addWidget(self._lbl_fb_label)
        self._lbl_fb_value = QLabel(str(self.fb_rpm))
        self._lbl_fb_value.setStyleSheet(
            "color: #FFD66B; font-size: 14px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_fb_value)

        lay.addSpacing(6)
        self._lbl_rot_label = QLabel("ROT RPM:")
        lay.addWidget(self._lbl_rot_label)
        self._lbl_rot_value = QLabel(str(self.rot_rpm))
        self._lbl_rot_value.setStyleSheet(
            "color: #FFD66B; font-size: 14px; font-weight: bold;"
        )
        lay.addWidget(self._lbl_rot_value)

        lay.addSpacing(6)
        lay.addWidget(QLabel("Turn Ratio:"))
        self._spin_turn_ratio = QDoubleSpinBox()
        self._spin_turn_ratio.setRange(0.0, 1.0)
        self._spin_turn_ratio.setSingleStep(0.05)
        self._spin_turn_ratio.setDecimals(2)
        self._spin_turn_ratio.setValue(self.turn_ratio)
        self._spin_turn_ratio.setFixedWidth(60)
        self._style_turn_ratio_spinbox()
        self._spin_turn_ratio.valueChanged.connect(self._on_turn_ratio_spin_changed)
        lay.addWidget(self._spin_turn_ratio)

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
        lay.setContentsMargins(6, 2, 6, 2)
        lay.setSpacing(6)

        # -- Left: three LEDs (red / yellow / green) ----------------------
        leds_col = QVBoxLayout()
        leds_col.setContentsMargins(0, 0, 0, 0)
        leds_col.setSpacing(3)
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
        right_col.setSpacing(3)

        self._lbl_op_mode_status = QLabel("DISARM")
        self._lbl_op_mode_status.setAlignment(Qt.AlignCenter)
        self._lbl_op_mode_status.setFixedHeight(28)
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

    def _build_mobility_mode_group(self) -> QGroupBox:
        grp = QGroupBox("Mobility Mode")
        grp.setStyleSheet("""
            QGroupBox {
                padding-bottom: 0px;
                margin-bottom: 0px;
            }
        """)
        lay = QHBoxLayout(grp)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(2)

        self._mobility_modes = [
            "Mode 1", "Mode 2", "Mode 3", "Mode 4", "Mode 5",
            "Mode 6", "Mode 7", "Mode 8", "Mode 9", "Mode 10"
        ]
        self._mobility_mode_config = {
            "Mode 1": {"fb_rpm": 30, "rot_rpm": 90},
            "Mode 2": {"fb_rpm": 60, "rot_rpm": 100},
            "Mode 3": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 4": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 5": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 6": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 7": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 8": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 9": {"fb_rpm": 30, "rot_rpm": 100},
            "Mode 10": {"fb_rpm": 30, "rot_rpm": 100},
        }
        self._mobility_mode_buttons = []
        shortcut_keys = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"]
        for i, mode in enumerate(self._mobility_modes):
            btn = QPushButton(mode)
            btn.setCheckable(True)
            btn.setShortcut(QKeySequence(shortcut_keys[i]))
            btn.setStyleSheet("""
                QPushButton {
                    padding: 2px 4px;
                    font-size: 10px;
                    min-width: 50px;
                    min-height: 20px;
                    max-height: 24px;
                }
                QPushButton:checked {
                    background-color: palette(highlight);
                    color: palette(highlighted-text);
                }
            """)
            btn.clicked.connect(lambda checked, m=mode: self._set_mobility_mode(m))
            self._mobility_mode_buttons.append(btn)
            lay.addWidget(btn)

        return grp

    def _set_mobility_mode(self, mode: str):
        for btn in self._mobility_mode_buttons:
            btn.setChecked(btn.text() == mode)
        config = self._mobility_mode_config.get(mode)
        if config:
            self.fb_rpm = config["fb_rpm"]
            self.rot_rpm = config["rot_rpm"]
            self._lbl_fb_value.setText(str(self.fb_rpm))
            self._lbl_rot_value.setText(str(self.rot_rpm))
        self._log_info(f"[MOBILITY] Mode set to: {mode} (FB RPM: {self.fb_rpm}, Rot RPM: {self.rot_rpm})")

    def keyPressEvent(self, event):
        key_map = {
            Qt.Key_1: 0, Qt.Key_2: 1, Qt.Key_3: 2, Qt.Key_4: 3, Qt.Key_5: 4,
            Qt.Key_6: 5, Qt.Key_7: 6, Qt.Key_8: 7, Qt.Key_9: 8, Qt.Key_0: 9,
        }
        idx = key_map.get(event.key())
        if idx is not None and idx < len(self._mobility_modes):
            self._set_mobility_mode(self._mobility_modes[idx])
        else:
            super().keyPressEvent(event)

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
        self._motor_table.verticalHeader().setDefaultSectionSize(22)
        self._motor_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._motor_table.setFocusPolicy(Qt.NoFocus)
        self._motor_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._motor_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._motor_table.setMaximumHeight(120)
        self._motor_table.setMinimumHeight(120)
        lay.addWidget(self._motor_table)

        # -- Settings / Faults buttons --------------------------------------
        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.setSpacing(4)
        btn_row.addStretch()

        self._btn_motor_faults = QPushButton("Faults")
        self._btn_motor_faults.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        self._btn_motor_faults.clicked.connect(self._open_fault_codes_dialog)
        btn_row.addWidget(self._btn_motor_faults)

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
        self._imu_grp = grp
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
        self._imu_table.setMaximumHeight(120)
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



    def _set_imu_cell(self, row: int, col: int, text: str):
        item = self._imu_table.item(row, col)
        if item is not None:
            item.setText(text)

    # -- Manipulation Arm Telemetry ----------------------------------------

    def _build_arm_telemetry_group(self) -> QGroupBox:
        grp = QGroupBox("Manipulation Arm Telemetry")
        self._arm_grp = grp
        grp.setStyleSheet("""
            QGroupBox {
                padding-bottom: 0px;
                margin-bottom: 0px;
            }
        """)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 0)
        lay.setSpacing(0)

        num_rows = len(self.ARM_JOINTS)
        num_cols = len(self.ARM_COL_HEADERS)
        self._arm_table = QTableWidget(num_rows, num_cols)
        self._arm_table.setHorizontalHeaderLabels(self.ARM_COL_HEADERS)
        headers = self._arm_table.horizontalHeader()
        if headers:
            headers.setSectionResizeMode(self.ARM_COL["Axis"], QHeaderView.Stretch)
            for col_idx in range(1, num_cols):
                headers.setSectionResizeMode(col_idx, QHeaderView.ResizeToContents)

        col = self.ARM_COL
        for row, joint in enumerate(self.ARM_JOINTS):
            st = self._arm_state[joint]
            self._arm_table.setItem(row, col["Axis"], QTableWidgetItem(self.ARM_JOINT_LABELS[joint]))
            self._arm_table.setItem(row, col["Degree"], QTableWidgetItem(st["degree"]))
            self._arm_table.setItem(row, col["Tgt"], QTableWidgetItem(st["tgt"]))
            self._arm_table.setItem(row, col["Dir"], QTableWidgetItem(st["dir"]))
            self._arm_table.setItem(row, col["PWM"], QTableWidgetItem(st["pwm"]))
            self._arm_table.setItem(row, col["Brake"], QTableWidgetItem(st["brake"]))
            self._arm_table.setItem(row, col["Stop"], QTableWidgetItem(st["stop"]))
            self._arm_table.setItem(row, col["Limit"], QTableWidgetItem(st["limit"]))
            self._arm_table.setItem(row, col["Sens"], QTableWidgetItem(st["sens"]))
            self._arm_table.setItem(row, col["Fault"], QTableWidgetItem(st["fault"]))

        self._arm_table.setVerticalHeaderLabels([])
        self._arm_table.verticalHeader().setVisible(False)
        self._arm_table.verticalHeader().setDefaultSectionSize(28)
        self._arm_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._arm_table.setFocusPolicy(Qt.NoFocus)
        self._arm_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._arm_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table_height = (num_rows + 1) * 28
        self._arm_table.setMinimumHeight(table_height)
        self._arm_table.setMinimumWidth(620)
        lay.addWidget(self._arm_table)

        # Settings + Refresh params + Polling toggle buttons
        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.setSpacing(4)
        self._btn_arm_poll = QPushButton("Start Arm Polling")
        self._btn_arm_poll.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        self._btn_arm_poll.setCheckable(True)
        self._btn_arm_poll.toggled.connect(self._arm_polling_toggled)
        btn_row.addWidget(self._btn_arm_poll)
        btn_row.addStretch()
        btn_arm_settings = QPushButton("Settings")
        btn_arm_settings.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        btn_arm_settings.clicked.connect(self._open_arm_settings)
        btn_row.addWidget(btn_arm_settings)
        btn_arm_refresh = QPushButton("Refresh Params")
        btn_arm_refresh.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        btn_arm_refresh.clicked.connect(self._arm_refresh_params)
        btn_row.addWidget(btn_arm_refresh)
        lay.addLayout(btn_row)

        return grp

    def _set_arm_cell(self, joint: str, col_name: str, text: str, color: str | None = None):
        row = self.ARM_JOINTS.index(joint)
        col = self.ARM_COL[col_name]
        item = self._arm_table.item(row, col)
        if item is not None:
            item.setText(text)
            if color:
                item.setForeground(QColor(color))

    def _arm_refresh_params(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("params")
            self._log_info("[ARM] Requesting params refresh")

    def _arm_poll_motors(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("get motors")

    def _arm_poll_sensors(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("get sensors")

    def _arm_poll_fault(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("get fault")

    def _arm_start_polling(self):
        self._telemetry_expected["ARM"] = True
        self._telemetry_expected_since["ARM"] = time.monotonic()
        self._arm_poll_motors_timer.start()
        self._arm_poll_sensors_timer.start()
        self._arm_poll_fault_timer.start()

    def _arm_stop_polling(self):
        self._telemetry_expected["ARM"] = False
        self._telemetry_expected_since["ARM"] = 0.0
        self._arm_poll_motors_timer.stop()
        self._arm_poll_sensors_timer.stop()
        self._arm_poll_fault_timer.stop()
        if hasattr(self, "_btn_arm_poll"):
            self._btn_arm_poll.setChecked(False)

    def _arm_polling_toggled(self, checked: bool):
        if checked:
            if not self._tcp_is_connected():
                self._btn_arm_poll.setChecked(False)
                self._log_warn("[ARM] Cannot start polling — not connected")
                return
            self._arm_start_polling()
            self._btn_arm_poll.setText("Stop Arm Polling")
            self._log_info("[ARM] Telemetry polling started")
        else:
            self._arm_stop_polling()
            self._btn_arm_poll.setText("Start Arm Polling")
            self._log_info("[ARM] Telemetry polling stopped")

    # ======================================================================
    #  Drill Telemetry (F401 M4/M5/M6 in DRILL mode)
    #  Compact panel beside the Manipulation Arm Telemetry panel:
    #    - Mode switch buttons (Manipulation / Drill / Safe / Stop All)
    #    - F401 mode chip
    #    - Drill telemetry table (Elevator L / Elevator R / Drill Motor)
    #    - Drill Activity info box
    #  All commands go through the centralized manipulation sender, which
    #  adds the H7 ``arm`` route before using _send_cmd().  The panel is
    #  telemetry + mode switching only - no drill settings controls here.
    # ======================================================================

    def _build_drill_telemetry_group(self) -> QGroupBox:
        """Build the compact Drill Telemetry panel.

        Mirrors the styling of the Manipulation Arm Telemetry group so both
        panels read consistently side-by-side.  The inner table is fixed
        width/height so the row does not grow uncontrollably.
        """
        grp = QGroupBox("Drill Telemetry")
        self._drill_grp = grp
        grp.setStyleSheet("""
            QGroupBox {
                padding-bottom: 0px;
                margin-bottom: 0px;
            }
        """)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 4, 6, 0)
        lay.setSpacing(4)

        # -- Mode switch buttons --------------------------------------------
        mode_row = QHBoxLayout()
        mode_row.setContentsMargins(0, 0, 0, 0)
        mode_row.setSpacing(4)

        def _mode_btn(label: str, cmd: str, *, danger=False) -> QPushButton:
            b = QPushButton(label)
            b.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
            b.clicked.connect(lambda: self._send_drill_cmd(cmd))
            return b

        b_manip = _mode_btn("ARM Mode", "mode arm confirm")
        b_drill = _mode_btn("DRILL Mode", "mode drill confirm")
        b_safe = _mode_btn("SAFE", "mode safe", danger=True)
        b_stopall = _mode_btn("STOP ALL", "stopall", danger=True)
        mode_row.addWidget(b_manip)
        mode_row.addWidget(b_drill)
        mode_row.addWidget(b_safe)
        mode_row.addWidget(b_stopall)
        mode_row.addStretch()
        self._btn_drill_poll = QPushButton("Start Drill Polling")
        self._btn_drill_poll.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        self._btn_drill_poll.setCheckable(True)
        self._btn_drill_poll.toggled.connect(self._drill_polling_toggled)
        mode_row.addWidget(self._btn_drill_poll)
        lay.addLayout(mode_row)

        # -- F401 mode chip --------------------------------------------------
        chip_row = QHBoxLayout()
        chip_row.setContentsMargins(0, 0, 0, 0)
        chip_row.setSpacing(4)
        self._drill_mode_chip = QLabel("F401 Mode: UNKNOWN")
        self._drill_mode_chip.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._drill_mode_chip.setStyleSheet(self._drill_chip_style("UNKNOWN"))
        chip_row.addWidget(self._drill_mode_chip)
        chip_row.addStretch()
        lay.addLayout(chip_row)

        # -- Drill telemetry table ------------------------------------------
        num_rows = len(self.DRILL_PARTS)
        num_cols = len(self.DRILL_COL_HEADERS)
        self._drill_table = QTableWidget(num_rows, num_cols)
        self._drill_table.setHorizontalHeaderLabels(self.DRILL_COL_HEADERS)
        headers = self._drill_table.horizontalHeader()
        if headers:
            col = self.DRILL_COL
            headers.setSectionResizeMode(col["Part"], QHeaderView.ResizeToContents)
            headers.setSectionResizeMode(col["Dir"], QHeaderView.ResizeToContents)
            headers.setSectionResizeMode(col["PWM"], QHeaderView.ResizeToContents)
            headers.setSectionResizeMode(col["Brake"], QHeaderView.ResizeToContents)
            headers.setSectionResizeMode(col["EN"], QHeaderView.ResizeToContents)
            headers.setSectionResizeMode(col["State"], QHeaderView.Stretch)
            headers.setSectionResizeMode(col["Fault"], QHeaderView.ResizeToContents)

        col = self.DRILL_COL
        for row, part in enumerate(self.DRILL_PARTS):
            st = self._drill_state["parts"][part]
            self._drill_table.setItem(row, col["Part"], QTableWidgetItem(self.DRILL_PART_LABELS[part]))
            self._drill_table.setItem(row, col["Dir"], QTableWidgetItem(st["dir"]))
            self._drill_table.setItem(row, col["PWM"], QTableWidgetItem(st["pwm"]))
            self._drill_table.setItem(row, col["Brake"], QTableWidgetItem(st["brake"]))
            self._drill_table.setItem(row, col["EN"], QTableWidgetItem(st["en"]))
            self._drill_table.setItem(row, col["State"], QTableWidgetItem(st["state"]))
            self._drill_table.setItem(row, col["Fault"], QTableWidgetItem(st["fault"]))

        self._drill_table.setVerticalHeaderLabels([])
        self._drill_table.verticalHeader().setVisible(False)
        self._drill_table.verticalHeader().setDefaultSectionSize(28)
        self._drill_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._drill_table.setFocusPolicy(Qt.NoFocus)
        self._drill_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._drill_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        drill_table_height = (num_rows + 1) * 28
        self._drill_table.setMinimumHeight(drill_table_height)
        self._drill_table.setMinimumWidth(430)
        lay.addWidget(self._drill_table)

        # -- Manual refresh buttons (debug only) -----------------------------
        refresh_row = QHBoxLayout()
        refresh_row.setContentsMargins(0, 0, 0, 0)
        refresh_row.setSpacing(4)
        b_get_mode = QPushButton("Get Mode")
        b_get_mode.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        b_get_mode.clicked.connect(lambda: self._send_drill_cmd("get mode"))
        b_get_motors = QPushButton("Get Motors")
        b_get_motors.setStyleSheet("QPushButton { padding: 0px; margin: 0px; }")
        b_get_motors.clicked.connect(lambda: self._send_drill_cmd("get motors"))
        refresh_row.addWidget(b_get_mode)
        refresh_row.addWidget(b_get_motors)
        refresh_row.addStretch()
        lay.addLayout(refresh_row)

        # -- Drill Activity info box ---------------------------------------
        self._drill_activity_box = QLabel("NO DATA")
        self._drill_activity_box.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._drill_activity_box.setWordWrap(True)
        self._drill_activity_box.setMinimumHeight(28)
        self._drill_activity_box.setStyleSheet(self._drill_activity_style("NO DATA"))
        lay.addWidget(self._drill_activity_box)

        lay.addStretch()
        return grp

    def _drill_chip_style(self, mode: str) -> str:
        """Style the F401 mode chip using the active theme palette."""
        c = self._colors()
        color_by_mode = {
            "SAFE": c.get("warning", c["text_muted"]),
            "ARM":  c.get("accent_gold", c["text"]),
            "DRILL": c.get("success_bright", c["text"]),
            "UNKNOWN": c["text_muted"],
        }
        fg = color_by_mode.get(mode, c["text_muted"])
        return (f"color: {fg}; font-size: 11px; font-weight: bold;"
                f" background: {c['bg_input']}; border: 1px solid {c['border']};"
                f" border-radius: 8px; padding: 2px 8px;")

    def _drill_activity_style(self, activity: str) -> str:
        """Style the Drill Activity box using the active theme palette.

        - neutral/idle: muted text
        - active movement/digging: success / accent
        - warning / mismatch: warning
        - fault: danger
        """
        c = self._colors()
        a = (activity or "").upper()
        warning_words = ("MISMATCH", "WARNING")
        fault_words = ("FAULT",)
        if any(w in a for w in fault_words):
            fg, bg, bd = c["danger_bright"], c["bg_input"], c["danger"]
        elif any(w in a for w in warning_words):
            fg, bg, bd = c["warning"], c["bg_input"], c["warning"]
        elif ("IDLE" in a or "SAFE" == a or "MANIPULATION" in a
              or "UNKNOWN" in a or "NO DATA" in a
              or "WAITING" in a):
            fg, bg, bd = c["text_muted"], c["bg_input"], c["border"]
        else:
            # Active: ELEVATOR UP/DOWN, DRILL DIGGING/EXTRACTING/ACTIVE, ...
            fg = c.get("success_bright", c["accent_gold"])
            bg = c["bg_input"]
            bd = c.get("success", c["accent_gold"])
        return (f"color: {fg}; font-size: 12px; font-weight: bold;"
                f" background: {bg}; border: 1px solid {bd};"
                f" border-radius: 6px; padding: 4px 8px;")

    def _send_drill_cmd(self, cmd: str):
        """Send a drill / mode command through the H7 manipulation route.

        A-side effect: track the last GUI-commanded drill activity so the
        Drill Activity box can fall back to it when the ``get motors``
        direction naming is not enough to classify UP/DOWN/DIG/EXTRACT.
        """
        self._drill_track_command(cmd)
        if not self._tcp_is_connected():
            self._log_warn(f"[DRILL] Not sent (not connected): {cmd}")
            return
        self._send_manipulation_cmd(cmd)

    def _drill_track_command(self, cmd: str):
        """Map the last GUI-commanded drill command to an activity hint."""
        parts = cmd.strip().split()
        if not parts:
            return
        kw = parts[0].lower()
        if kw == "elv":
            if len(parts) >= 2:
                sub = parts[1].lower()
                if sub == "up":
                    self._drill_state["last_commanded_activity"] = "ELEVATOR UP"
                elif sub == "down":
                    self._drill_state["last_commanded_activity"] = "ELEVATOR DOWN"
                elif sub == "stop":
                    self._drill_state["last_commanded_activity"] = "DRILL IDLE"
        elif kw == "drill":
            if len(parts) >= 2:
                sub = parts[1].lower()
                if sub == "dig":
                    self._drill_state["last_commanded_activity"] = "DRILL DIGGING"
                elif sub == "extract":
                    self._drill_state["last_commanded_activity"] = "DRILL EXTRACTING"
                elif sub == "stop":
                    self._drill_state["last_commanded_activity"] = "DRILL IDLE"
        elif kw in ("stop", "stopall"):
            self._drill_state["last_commanded_activity"] = "DRILL IDLE"
        elif kw == "mode":
            if len(parts) >= 2:
                sub = parts[1].lower()
                if sub == "safe":
                    self._drill_state["last_commanded_activity"] = "SAFE"
                elif sub == "arm":
                    self._drill_state["last_commanded_activity"] = "MANIPULATION MODE"
                elif sub == "drill":
                    # Mode change itself does not imply movement.
                    self._drill_state["last_commanded_activity"] = "DRILL IDLE"
        self._update_drill_activity_box()

    def _drill_poll_motors(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("get motors")

    def _drill_poll_fault(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("get fault")

    def _drill_poll_mode(self):
        if self._tcp_is_connected():
            self._send_manipulation_cmd("get mode")

    def _drill_start_polling(self):
        self._telemetry_expected["DRILL"] = True
        self._telemetry_expected_since["DRILL"] = time.monotonic()
        self._drill_poll_motors_timer.start()
        self._drill_poll_fault_timer.start()
        self._drill_poll_mode_timer.start()

    def _drill_stop_polling(self):
        self._telemetry_expected["DRILL"] = False
        self._telemetry_expected_since["DRILL"] = 0.0
        self._drill_poll_motors_timer.stop()
        self._drill_poll_fault_timer.stop()
        self._drill_poll_mode_timer.stop()
        if hasattr(self, "_btn_drill_poll"):
            self._btn_drill_poll.setChecked(False)

    def _drill_polling_toggled(self, checked: bool):
        if checked:
            if not self._tcp_is_connected():
                self._btn_drill_poll.setChecked(False)
                self._log_warn("[DRILL] Cannot start polling — not connected")
                return
            self._drill_start_polling()
            self._btn_drill_poll.setText("Stop Drill Polling")
            self._log_info("[DRILL] Telemetry polling started")
        else:
            self._drill_stop_polling()
            self._btn_drill_poll.setText("Start Drill Polling")
            self._log_info("[DRILL] Telemetry polling stopped")

    def _set_drill_cell(self, part: str, col_name: str, text: str,
                         color: str | None = None):
        row = self.DRILL_PARTS.index(part)
        col = self.DRILL_COL[col_name]
        item = self._drill_table.item(row, col)
        if item is not None:
            item.setText(text)
            if color:
                item.setForeground(QColor(color))

    def _update_drill_table(self):
        """Refresh the drill telemetry table from ``self._drill_state``."""
        c = self._colors()
        col = self.DRILL_COL
        for part in self.DRILL_PARTS:
            st = self._drill_state["parts"][part]
            self._set_drill_cell(part, "Dir", st["dir"])
            self._set_drill_cell(part, "PWM", st["pwm"])
            self._set_drill_cell(part, "Brake", st["brake"],
                                 c["danger"] if st["brake"] == "ON" else None)
            self._set_drill_cell(part, "EN", st["en"],
                                 c["success_bright"] if st["en"] == "ON" else None)
            self._set_drill_cell(part, "State", st["state"])
            fault = st["fault"]
            fault_color = None
            if fault == "FAULT":
                fault_color = c["danger"]
            elif fault == "MISMATCH":
                fault_color = c["warning"]
            elif fault == "OK":
                fault_color = c["success_bright"]
            self._set_drill_cell(part, "Fault", fault, fault_color)

    def _update_drill_mode_chip(self):
        mode = self._drill_state["mode"]
        self._drill_mode_chip.setText(f"F401 Mode: {mode}")
        self._drill_mode_chip.setStyleSheet(self._drill_chip_style(mode))

    def _update_drill_activity_box(self):
        """Decide the current Drill Activity text and re-style the box."""
        a = self._drill_compute_activity()
        self._drill_state["activity"] = a
        if self._drill_state["heartbeat_fault"]:
            a = "FAULT"
        self._drill_activity_box.setText(a)
        self._drill_activity_box.setStyleSheet(self._drill_activity_style(a))

    def _drill_compute_activity(self) -> str:
        """Classify the current drill-system activity from state + commands.

        See the task spec for the full decision table.  Returns one of:
        NO DATA / SAFE / MANIPULATION MODE / DRILL IDLE / ELEVATOR UP
        / ELEVATOR DOWN / ELEVATOR L ACTIVE / ELEVATOR R ACTIVE
        / ELEVATOR L/R MISMATCH / DRILL DIGGING / DRILL EXTRACTING
        / DRILL ACTIVE / FAULT / UNKNOWN.
        """
        st = self._drill_state
        mode = st["mode"]
        if mode == "SAFE":
            return "SAFE"
        if mode == "ARM":
            return "MANIPULATION MODE"
        if st["heartbeat_fault"]:
            return "FAULT"
        # DRILL or UNKNOWN -> look at parts.
        parts = st["parts"]
        el = parts["elevator_l"]
        er = parts["elevator_r"]
        dr = parts["drill"]

        def _active(p: dict) -> bool:
            return (p["dir"] != "STOP") or (int(_safe_int(p["pwm"])) > 0)

        el_act = _active(el)
        er_act = _active(er)
        dr_act = _active(dr)

        if not el_act and not er_act and not dr_act:
            if mode == "DRILL":
                return "DRILL IDLE"
            # Mode not yet confirmed and no telemetry activity -> show a
            # clear "no data yet" hint instead of the opaque "UNKNOWN".
            if mode == "UNKNOWN":
                return "NO DATA"
            return "UNKNOWN"

        # Drill motor first - it has its own dig/extract classification.
        if dr_act:
            d_dir = dr["dir"].upper()
            if d_dir in ("DIG", "FWD"):
                return "DRILL DIGGING"
            if d_dir in ("EXTRACT", "BWD", "REV"):
                return "DRILL EXTRACTING"
            # Fall back to last GUI-commanded drill activity if available.
            last = st["last_commanded_activity"].upper()
            if "DIGGING" in last:
                return "DRILL DIGGING"
            if "EXTRACTING" in last:
                return "DRILL EXTRACTING"
            return "DRILL ACTIVE"

        # Elevator pair logic.
        if el_act and er_act:
            # Both active - classify direction + mismatch check.
            if (el["dir"] != er["dir"]
                    or el["en"] != er["en"]
                    or el["brake"] != er["brake"]):
                return "ELEVATOR L/R MISMATCH"
            d = el["dir"].upper()
            # In elevator land prefer the last GUI-commanded hint.
            last = st["last_commanded_activity"].upper()
            if "UP" in last:
                return "ELEVATOR UP"
            if "DOWN" in last:
                return "ELEVATOR DOWN"
            if d in ("UP", "FWD"):
                return "ELEVATOR UP"
            if d in ("DOWN", "BWD", "REV"):
                return "ELEVATOR DOWN"
            return "DRILL ACTIVE"
        if el_act:
            return "ELEVATOR L ACTIVE"
        if er_act:
            return "ELEVATOR R ACTIVE"
        return "DRILL ACTIVE"

    # -- Drill parsers --------------------------------------------------------

    def _parse_drill_get_motors_line(self, line: str) -> bool:
        """Parse ``M4/M5/M6`` ``DIR= PWM= BRAKE= EN= ...`` lines.

        Tolerant search: any line that contains a leading ``M4``/``M5``/``M6``
        token followed by KEY=VALUE pairs is accepted.  Unknown fields are
        ignored.  Returns ``True`` if a recognised motor line updated the
        drill state, ``False`` otherwise.  This is invoked from
        ``_arm_parse_motors`` for any M4/M5/M6 line it already matched, and
        also directly from ``_on_rx_line`` as a fallback parser for variations
        (e.g. extra keys) the manipulation regex might miss.
        """
        m = re.search(r"(^|\s)M([456])(?=\s|$|[^0-9])", line)
        if not m:
            return False
        motor_num = int(m.group(2))
        motor_tag = f"M{motor_num}"
        part = next((p for p, mm in self.DRILL_MOTOR_MAP.items()
                     if mm == motor_tag), None)
        if part is None:
            return False
        # Extract all KEY=VALUE tokens in the line.
        kv: dict[str, str] = {}
        for tok in line.split():
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            kv[k.strip().upper()] = v.strip()
        if not kv:
            return False

        c = self._colors()
        st = self._drill_state["parts"][part]
        if "DIR" in kv:
            raw_dir = kv["DIR"].upper()
            st["dir"] = raw_dir
        if "PWM" in kv:
            st["pwm"] = str(_safe_int(kv["PWM"], 0))
        if "BRAKE" in kv:
            st["brake"] = "ON" if kv["BRAKE"].upper() == "ON" else "OFF"
        if "EN" in kv:
            st["en"] = "ON" if kv["EN"].upper() == "ON" else "OFF"

        # Derive a simple per-part state.
        pwm_val = int(_safe_int(st["pwm"], 0))
        if st["dir"] == "STOP" and pwm_val <= 0:
            st["state"] = "IDLE"
        elif st["brake"] == "ON":
            st["state"] = "STOPPED"
        elif pwm_val > 0:
            st["state"] = "MOVING"
        else:
            st["state"] = "UNKNOWN"

        self._touch_drill_freshness()
        self._update_drill_table()
        self._update_drill_activity_box()
        return True

    def _parse_drill_mode_line(self, line: str) -> bool:
        """Parse F401 ``get mode`` / mode-change confirmation lines.

        Recognised (case-insensitive; F401 vocabulary is SAFE/ARM/DRILL only):
            MODE SAFE / MODE ARM / MODE DRILL          (spaced)
            MODE=SAFE / MODE=ARM / MODE=DRILL          (key=value)
            OK MODE SAFE / OK MODE ARM / OK MODE DRILL (prefixed ack)
            OK MODE=SAFE / OK MODE=ARM / OK MODE=DRILL (prefixed ack, kv)
            SAFE active / ARM active / DRILL active    (mode confirm)
            [MODE] SAFE / [MODE] ARM / [MODE] DRILL    (tagged log lines)
            ... SAFE mode / ... ARM mode / ... DRILL mode (phrase)
        Lines that don't look like an F401 mode line return ``False`` so the
        H7 operating-mode parser keeps ownership of DISARM/MANUAL/AUTONOMOUS:
        the F401 vocabulary here intentionally excludes those tokens.
        """
        upper = line.upper()
        mode = None
        # Key=value: ``MODE=SAFE`` / ``OK MODE=SAFE`` (with optional spaces
        # around ``=``) and the spaced variant ``MODE SAFE`` / ``OK MODE SAFE``.
        m = re.search(r"\bMODE\s*[=\s]\s*(SAFE|ARM|DRILL)\b", upper)
        if m:
            mode = m.group(1)
        else:
            # ``<NAME> active`` confirmation (F401 only).
            m = re.search(r"\b(SAFE|ARM|DRILL)\s+ACTIVE\b", upper)
            if m:
                mode = m.group(1)
            else:
                # Tagged log line: ``[MODE] SAFE`` / ``[MODE] ARM`` / ...
                m = re.search(r"\[MODE\]\s+(SAFE|ARM|DRILL)\b", upper)
                if m:
                    mode = m.group(1)
                else:
                    # Phrase: ``... SAFE mode`` / ``... ARM mode`` / ...
                    m = re.search(r"\b(SAFE|ARM|DRILL)\s+MODE\b", upper)
                    if m:
                        mode = m.group(1)
        if mode is None:
            return False
        self._drill_state["mode"] = mode
        self._touch_drill_freshness()
        self._update_drill_mode_chip()
        self._update_drill_activity_box()
        return True

    def _parse_drill_fault_line(self, line: str) -> bool:
        """Parse fault indicators relevant to the drill system.

        Currently detects heartbeat-timeout fault markers and clears the
        heartbeat flag otherwise.  Returns ``True`` if a relevant drill
        fault line was recognised.
        """
        upper = line.upper()
        if "HEARTBEAT_TIMEOUT" in upper:
            self._drill_state["heartbeat_fault"] = True
            # Mark all drill parts as FAULT for visibility.
            for p in self.DRILL_PARTS:
                self._drill_state["parts"][p]["fault"] = "FAULT"
            self._update_drill_table()
            self._update_drill_activity_box()
            return True
        return False

    def _drill_clear_heartbeat_fault(self):
        """Clear the heartbeat-fault flag and reset per-part fault to OK."""
        if self._drill_state["heartbeat_fault"]:
            self._drill_state["heartbeat_fault"] = False
            for p in self.DRILL_PARTS:
                self._drill_state["parts"][p]["fault"] = "OK"
            self._update_drill_table()
            self._update_drill_activity_box()

    def _arm_parse_motors(self, line: str) -> bool:
        m = re.match(r"^M(\d)\s+DIR=(\w+)\s+PWM=(\d+)\s+BRAKE=(\w+)", line)
        if not m:
            return False
        motor_num = int(m.group(1))
        dir_val = m.group(2)
        pwm_val = m.group(3)
        brake_val = m.group(4)
        motor_tag = f"M{motor_num}"
        c = self._colors()
        dir_display = {"FWD": "FWD", "BWD": "BWD", "STOP": "STOP", "HOLD": "HOLD"}.get(dir_val, dir_val)
        brake_display = "ON" if brake_val.upper() == "ON" else "OFF"
        brake_color = c["danger"] if brake_display == "ON" else None
        for joint in self.ARM_JOINTS:
            if joint in ("J4", "J5"):
                continue
            if self.ARM_MOTOR_MAP.get(joint) == motor_tag:
                self._set_arm_cell(joint, "Dir", dir_display)
                self._set_arm_cell(joint, "PWM", pwm_val)
                self._set_arm_cell(joint, "Brake", brake_display, brake_color)
                self._arm_state[joint]["dir"] = dir_display
                self._arm_state[joint]["pwm"] = pwm_val
                self._arm_state[joint]["brake"] = brake_display
        m4_state = None
        m5_state = None
        for joint in self.ARM_JOINTS:
            if self.ARM_MOTOR_MAP.get(joint) == "M4":
                m4_state = self._arm_state.get("J4")
            if self.ARM_MOTOR_MAP.get(joint) == "M5":
                m5_state = self._arm_state.get("J5")
        if motor_num == 4:
            self._arm_state.setdefault("J4", {})["pwm_m4"] = pwm_val
            self._arm_state.setdefault("J4", {})["dir_m4"] = dir_val
            self._arm_state.setdefault("J5", {})["pwm_m4"] = pwm_val
            self._arm_state.setdefault("J5", {})["dir_m4"] = dir_val
        elif motor_num == 5:
            self._arm_state.setdefault("J4", {})["pwm_m5"] = pwm_val
            self._arm_state.setdefault("J4", {})["dir_m5"] = dir_val
            self._arm_state.setdefault("J5", {})["pwm_m5"] = pwm_val
            self._arm_state.setdefault("J5", {})["dir_m5"] = dir_val
        if motor_num in (4, 5):
            for wrist_joint in ("J4", "J5"):
                st = self._arm_state[wrist_joint]
                m4_pwm = int(st.get("pwm_m4", "0"))
                m5_pwm = int(st.get("pwm_m5", "0"))
                max_pwm = max(m4_pwm, m5_pwm)
                self._set_arm_cell(wrist_joint, "PWM", str(max_pwm))
                st["pwm"] = str(max_pwm)
                m4_dir = st.get("dir_m4", "STOP")
                m5_dir = st.get("dir_m5", "STOP")
                if m4_dir == "STOP" and m5_dir == "STOP":
                    wrist_dir = "STOP"
                elif wrist_joint == "J4":
                    if m4_dir == m5_dir:
                        wrist_dir = m4_dir
                    else:
                        wrist_dir = m4_dir
                else:
                    if m4_dir != m5_dir:
                        wrist_dir = m4_dir
                    else:
                        wrist_dir = "STOP"
                self._set_arm_cell(wrist_joint, "Dir", wrist_dir)
                st["dir"] = wrist_dir
                self._set_arm_cell(wrist_joint, "Brake", brake_display, brake_color)
                st["brake"] = brake_display
        self._touch_arm_freshness()
        return True

    def _arm_parse_sensors(self, line: str) -> bool:
        m = re.match(r"^SENSOR\s+J(\d)\s+CH=(\d+)\s+DEG=([-\d.]+)\s+VEL_DPS=([-\d.]+)\s+STATUS=(\w+)", line)
        if not m:
            return False
        joint_num = int(m.group(1))
        deg_val = m.group(3)
        status_val = m.group(5)
        joint = f"J{joint_num}"
        if joint not in self.ARM_JOINTS:
            return False
        c = self._colors()
        self._arm_sensors_seen[joint] = True
        self._set_arm_cell(joint, "Degree", f"{float(deg_val):.1f}\u00b0")
        self._arm_state[joint]["degree"] = deg_val
        sens_map = {
            "MAGNET_OK": ("OK", c["success_bright"]),
            "MAGNET_STRONG": ("OK", c["success_bright"]),
            "MAGNET_WEAK": ("WEAK", c["warning"]),
            "NO_MAGNET": ("NO", c["danger"]),
        }
        sens_text, sens_color = sens_map.get(status_val, (status_val, None))
        if "ERR=READ_FAIL" in status_val:
            sens_text, sens_color = "ERR", c["danger"]
        self._set_arm_cell(joint, "Sens", sens_text, sens_color)
        self._arm_state[joint]["sens"] = sens_text
        if self._arm_tgt_angle[joint] is not None:
            self._set_arm_cell(joint, "Tgt", f"{self._arm_tgt_angle[joint]:.1f}\u00b0")
        return True

    def _arm_parse_fault(self, line: str) -> bool:
        c = self._colors()
        m_lim = re.search(r"LIMIT_BITS=(\d+)", line)
        if m_lim:
            bits = int(m_lim.group(1))
            for i, joint in enumerate(("J1", "J2", "J3")):
                if self.ARM_HAS_LIMIT[joint]:
                    lim_active = (bits >> i) & 1
                    if lim_active:
                        self._set_arm_cell(joint, "Limit", "LIM", c["danger"])
                        self._arm_state[joint]["limit"] = "LIM"
                    else:
                        self._set_arm_cell(joint, "Limit", "OK", c["success_bright"])
                        self._arm_state[joint]["limit"] = "OK"
            # F401 fault reports commonly also include SETTINGS_DIRTY / mode /
            # heartbeat status fields on the same line.  Parse them here so
            # the dialog status hint updates even on a single multi-key line.
            self._arm_parse_fault_extras(line)
            return True
        m_bp = re.search(r"BRAKEPOINT_BITS=(\d+)", line)
        if m_bp:
            bits = int(m_bp.group(1))
            for i, joint in enumerate(("J1", "J2", "J3")):
                bp_active = (bits >> i) & 1
                if bp_active:
                    self._set_arm_cell(joint, "Fault", "BP", c["warning"])
                    self._arm_state[joint]["fault"] = "BP"
            self._arm_parse_fault_extras(line)
            return True
        m_hb = re.search(r"HEARTBEAT_TIMEOUT", line)
        if m_hb:
            self._arm_heartbeat_active = True
            for joint in self.ARM_JOINTS:
                self._set_arm_cell(joint, "Fault", "HB", c["danger"])
                self._arm_state[joint]["fault"] = "HB"
            self._arm_parse_fault_extras(line)
            return True
        # Fall-through: a fault report may include SETTINGS_DIRTY without
        # LIMIT_BITS / BRAKEPOINT_BITS / HEARTBEAT_TIMEOUT.  Detect it here
        # so the dialog status hint still updates for partial reports.
        if "SETTINGS_DIRTY" in line:
            self._arm_parse_fault_extras(line)
            return True
        return False

    def _arm_parse_fault_extras(self, line: str):
        """Parse extra fault report KEY=VALUE fields (SETTINGS_DIRTY etc).

        Called whenever a fault report prefix matches.  Updates the open
        ``ManipulationArmSettingsDialog`` (if any) with dirty / save hints.
        Missing fields are silently ignored.
        """
        m_dirty = re.search(r"SETTINGS_DIRTY=(\d+)", line)
        if m_dirty:
            try:
                dirty = int(m_dirty.group(1)) != 0
            except ValueError:
                dirty = False
            dlg = self._arm_settings_dialog
            if dlg is not None and dlg.isVisible():
                if dirty:
                    dlg.notify_settings_dirty()

    def _arm_parse_params(self, line: str) -> bool:
        """Parse F401 ``params`` output and update the GUI.

        Firmware emits one line per joint shaped like:

            J1 STOPMODE=BRAKE INVERT=0 MAXPWM=255 DEFAULT=100 \\
                POSGAIN=1000 NEGGAIN=1000 DEAD=50 KP=800 KD=40 \\
                MINPWM=60 TOL=2 MIN=-90 MAX=90 LIMITS=1 \\
                BRAKEPOINT=OFF KEYFWD=-1 KEYBACK=-1 AS5600=2

        Any subset of these keys is accepted; unknown / malformed keys are
        silently ignored so a partial or extended F401 output cannot crash
        the parser.  The existing Manipulation Arm Telemetry Stop column is
        still updated (existing behaviour), and the open
        ``ManipulationArmSettingsDialog`` (if any) is populated from the
        parsed dict.
        """
        m = re.match(r"^J(\d)\s+(?P<rest>[\w=.\-]+(?:\s+[\w=.\-]+)*)\s*$", line)
        if not m:
            return False
        joint_num = int(m.group(1))
        joint = f"J{joint_num}"
        if joint not in self.ARM_JOINTS:
            return False
        rest = m.group("rest")
        # Split into KEY=VALUE tokens.  Tokens without ``=`` are ignored.
        parsed: dict[str, str] = {}
        for tok in rest.split():
            if "=" not in tok:
                continue
            key, value = tok.split("=", 1)
            key = key.strip().upper()
            value = value.strip()
            if key:
                parsed[key] = value
        if not parsed:
            return False

        c = self._colors()
        stopmode = parsed.get("STOPMODE")
        if stopmode:
            stop_map = {
                "COAST": "COAST", "BRAKE": "BRAKE", "HOLD": "HOLD",
                "HYBRID": "HYB",
            }
            display = stop_map.get(stopmode.upper(), stopmode)
            self._set_arm_cell(joint, "Stop", display)
            self._arm_state[joint]["stop"] = display

        # Persist the parsed dict and apply to the dialog if it is open.
        per_joint = dict(parsed)
        # Normalise KEYBACK->KEYBACK handling preserved as-is.
        self._arm_params_cache[joint] = per_joint
        dlg = self._arm_settings_dialog
        if dlg is not None and dlg.isVisible():
            dlg.apply_params({joint: per_joint})
        return True

    def _arm_update_fault_from_state(self):
        c = self._colors()
        for joint in self.ARM_JOINTS:
            fault = "OK"
            color = c["success_bright"]
            st = self._arm_state[joint]
            if st.get("sens") == "ERR":
                fault = "SENS"
                color = c["danger"]
            elif st.get("limit") == "LIM":
                fault = "LIM"
                color = c["danger"]
            elif st.get("fault") == "BP":
                fault = "BP"
                color = c["warning"]
            elif st.get("fault") == "HB":
                fault = "HB"
                color = c["danger"]
            if self._arm_heartbeat_active and fault == "OK":
                fault = "HB"
                color = c["danger"]
            self._set_arm_cell(joint, "Fault", fault, color)
            st["fault"] = fault

    def _arm_track_goto(self, joint: str, angle: float):
        self._arm_tgt_angle[joint] = angle
        self._set_arm_cell(joint, "Tgt", f"{angle:.1f}\u00b0")

    def _arm_track_stop(self, joint: str | None = None):
        if joint:
            self._arm_tgt_angle[joint] = None
            self._set_arm_cell(joint, "Tgt", "\u2014")
        else:
            for j in self.ARM_JOINTS:
                self._arm_tgt_angle[j] = None
                self._set_arm_cell(j, "Tgt", "\u2014")

    def _arm_process_rx(self, line: str):
        if self._arm_parse_sensors(line):
            self._touch_arm_freshness()
            self._arm_update_fault_from_state()
            # Mirror sensor status into the open Arm Settings dialog.
            self._arm_settings_dialog_update_sensor(line)
            return
        if self._arm_parse_fault(line):
            self._touch_arm_freshness()
            self._arm_update_fault_from_state()
            return
        if self._arm_parse_params(line):
            self._touch_arm_freshness()
            return
        # F401 save result / error detection for the Arm Settings dialog.
        self._arm_settings_dialog_check_save(line)

    def _arm_settings_dialog_update_sensor(self, line: str):
        """Mirror SENSOR J<n> ... STATUS=... into the open Arm Settings dialog."""
        dlg = self._arm_settings_dialog
        if dlg is None or not dlg.isVisible():
            return
        m = re.search(r"J(\d)\s.*STATUS=(\w+)", line)
        if not m:
            return
        joint = f"J{m.group(1)}"
        status_val = m.group(2)
        sens_map = {
            "MAGNET_OK":     "OK",
            "MAGNET_STRONG": "OK",
            "MAGNET_WEAK":   "WEAK",
            "NO_MAGNET":     "NO",
        }
        sens_text = sens_map.get(status_val, status_val)
        if "ERR=READ_FAIL" in status_val:
            sens_text = "ERR"
        dlg.update_sensor_status(joint, sens_text)

    def _arm_settings_dialog_check_save(self, line: str):
        """Detect F401 save success / failure lines and notify the dialog.

        Recognised strings (case-insensitive):
            OK SAVED_FLASH                - save success, clears local dirty
            ERR SAVE_REQUIRES_SAFE        - not in SAFE mode
            ERR SAVE_REQUIRES_STOPPED     - motors still moving
        """
        dlg = self._arm_settings_dialog
        if dlg is None or not dlg.isVisible():
            return
        upper = line.upper()
        if "OK SAVED_FLASH" in upper:
            self._log_info("[ARM-SAVE] OK SAVED_FLASH")
            dlg.notify_save_success()
        elif "ERR SAVE_REQUIRES_SAFE" in upper:
            self._log_err("[ARM-SAVE] ERR SAVE_REQUIRES_SAFE")
            dlg.notify_save_failure("ERR SAVE_REQUIRES_SAFE (move to SAFE mode first)")
        elif "ERR SAVE_REQUIRES_STOPPED" in upper:
            self._log_err("[ARM-SAVE] ERR SAVE_REQUIRES_STOPPED")
            dlg.notify_save_failure("ERR SAVE_REQUIRES_STOPPED (stop all motors first)")

    def _update_motion_indicator(self, direction: str | None):
        """Update the Motion badge in Rover Status.  direction is one of W/S/A/D/T/Y/G/H or None for IDLE."""
        mapping = {"W": "FORWARD", "S": "BACKWARD", "A": "LEFT", "D": "RIGHT", "T": "ARC-LEFT", "Y": "ARC-RIGHT", "G": "ARC-BK-LEFT", "H": "ARC-BK-RIGHT"}
        text = mapping.get(direction, "IDLE")
        c = self._colors()
        color = c['accent_gold_bright'] if text != "IDLE" else c['text_muted']
        self._lbl_qs_motion.setText(f"Motion: {text}")
        self._lbl_qs_motion.setStyleSheet(self._style_badge(color))

    def _build_console_group(self) -> QGroupBox:
        grp = QGroupBox("Console")
        lay = QVBoxLayout(grp)

        # -- GUI Console -----------------------------------------------
        self._lbl_gui_console_title = QLabel("GUI Console")
        self._lbl_gui_console_title.setStyleSheet(
            "color: #8E8E93; font-weight: bold; font-size: 13px;"
        )
        lay.addWidget(self._lbl_gui_console_title)

        self._gui_console = QTextEdit()
        self._gui_console.setReadOnly(True)
        self._gui_console.setStyleSheet(
            "QTextEdit { background-color: #0B0B0D; border: 1px solid #5F5A4A; "
            "border-radius: 4px; color: #8E8E93; "
            "font-family: 'Consolas', 'Courier New', monospace; font-size: 12px; }"
        )
        self._gui_console.document().setMaximumBlockCount(2000)
        lay.addWidget(self._gui_console)

        btn_clear_gui = QPushButton("Clear GUI Console")
        btn_clear_gui.clicked.connect(self._gui_console.clear)
        lay.addWidget(btn_clear_gui)

        return grp

    # ======================================================================
    #  Theme Management
    # ======================================================================
    #
    #  Theme switching is purely visual.  _toggle_theme() flips self.current_theme,
    #  regenerates the stylesheet, restyles every theme-aware inline style and
    #  repaints dynamic widgets to match the current state - WITHOUT touching
        #  runtime state (connection, values, operating mode, pending mode,
    #  console contents, motor/IMU tables).

    def _toggle_theme(self):
        """Switch between dark and light theme.  No network I/O."""
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

        # Ensure the main window palette background matches the theme so
        # the QMainWindow background-color from the stylesheet is consistent.
        pal = self.palette()
        pal.setColor(self.backgroundRole(), QColor(c['bg_main']))
        self.setPalette(pal)

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
        if self._tcp_is_connected():
            peer = self._tcp_socket.peerAddress().toString()
            port = self._tcp_socket.peerPort()
            self._lbl_qs_port.setText(f"Link: {peer}:{port}")
            self._lbl_qs_port.setStyleSheet(self._style_badge(c['accent_gold']))
        else:
            self._lbl_qs_port.setText("Link: Disconnected")
            self._lbl_qs_port.setStyleSheet(self._style_badge(c['danger']))
        # Mode label + value label (RPM gold / DUTY amber)
        if hasattr(self, "_lbl_mode"):
            self._style_mode_value_labels()
        # Turn Ratio spinbox
        if hasattr(self, "_spin_turn_ratio"):
            self._style_turn_ratio_spinbox()

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
        state = (self._tcp_socket.state()
                 if self._tcp_socket is not None
                 else QAbstractSocket.UnconnectedState)
        if state in (QAbstractSocket.ConnectedState,
                     QAbstractSocket.ClosingState):
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
        color = c['accent_gold'] if self._tcp_is_connected() else c['danger']
        self._lbl_status.setStyleSheet(
            f"color: {color}; font-weight: bold;"
        )

    def _style_console_widgets(self):
        """Re-style the GUI console and its section label for the active theme."""
        c = self._colors()

        # Keep references to the section labels so they can be re-themed.
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
            f"color: {mode_color}; font-size: 13px; font-weight: bold;"
        )
        self._lbl_fb_value.setStyleSheet(
            f"color: {value_color}; font-size: 14px; font-weight: bold;"
        )
        self._lbl_rot_value.setStyleSheet(
            f"color: {value_color}; font-size: 14px; font-weight: bold;"
        )

    def _style_turn_ratio_spinbox(self):
        """Re-style the Turn Ratio spinbox for the active theme."""
        c = self._colors()
        color = c['accent_gold_bright']
        bg = c['bg_input']
        border = c['border']
        self._spin_turn_ratio.setStyleSheet(
            f"QDoubleSpinBox {{ color: {color}; background-color: {bg}; "
            f"border: 1px solid {border}; border-radius: 4px; "
            f"padding: 2px 4px; font-size: 13px; font-weight: bold; }}"
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

    def _log_info(self, text: str):
        self._log_gui("[GUI]", text, self._colors()['text_muted'])

    def _log_err(self, text: str):
        self._log_gui("[GUI-ERROR]", text, self._colors()['danger'])

    def _log_warn(self, text: str):
        self._log_gui("[GUI-WARN]", text, self._colors()['warning'])

    # ======================================================================
    #  TCP Connection Management
    # ======================================================================

    def _init_tcp_socket(self):
        """Create and configure the QTcpSocket."""
        self._tcp_socket = QTcpSocket()
        self._tcp_socket.connected.connect(self._on_tcp_connected)
        self._tcp_socket.disconnected.connect(self._on_tcp_disconnected)
        self._tcp_socket.readyRead.connect(self._on_tcp_ready_read)
        self._tcp_socket.errorOccurred.connect(self._on_tcp_error)
        self._tcp_socket.stateChanged.connect(self._on_tcp_state_changed)

    def _tcp_is_connected(self) -> bool:
        """Authoritative TCP connection check."""
        return (self._tcp_socket is not None and
                self._tcp_socket.state() == QAbstractSocket.ConnectedState)

    def _is_current_tcp_session(self, session_id: int) -> bool:
        """Check whether *session_id* still matches the active TCP session.

        A physical ``ConnectedState`` check alone cannot distinguish an old
        session from a new one after a rapid reconnect.  This helper adds
        the generation check so stale callbacks are silently dropped.
        """
        return (
            session_id == self._tcp_session_id
            and self._tcp_is_connected()
        )

    def _toggle_connection(self):
        """Dispatch the connection button from the real QTcpSocket state."""
        state = self._tcp_socket.state()
        if state == QAbstractSocket.UnconnectedState:
            self._tcp_connect()
        elif state in (QAbstractSocket.HostLookupState,
                       QAbstractSocket.ConnectingState):
            self._cancel_tcp_connect()
        elif state == QAbstractSocket.ConnectedState:
            self._disconnect()
        # ClosingState deliberately ignores extra clicks.  The state-driven
        # UI disables the button while closure is in progress.

    def _tcp_connect(self):
        if (self._tcp_socket is None or
                self._tcp_socket.state() != QAbstractSocket.UnconnectedState):
            return

        host = self._host_edit.text().strip()
        port = self._port_spin.value()
        if not host:
            self._log_warn("No host specified.")
            return

        self._log_info(f"Connecting to {host}:{port}...")
        self._lbl_status.setText("Status: CONNECTING")
        self._lbl_status.setStyleSheet(
            f"color: {self._colors()['warning']}; font-weight: bold;"
        )
        self._tcp_rx_buffer.clear()
        self._tcp_session_id += 1
        self._tcp_attempt_session_id = self._tcp_session_id
        self._tcp_connected_session_id = None
        self._tcp_teardown_session_id = None
        self._tcp_teardown_reason = None
        self._tcp_teardown_detail = None
        self._tcp_prepared_session_id = None
        self._tcp_connect_timer.start()
        self._tcp_socket.connectToHost(host, port)

    def _begin_tcp_teardown(self, reason: str, detail: str | None = None) -> int:
        """Assign one reason and invalidate one active attempt/session once."""
        if self._tcp_teardown_reason is not None:
            if self._tcp_teardown_session_id is not None:
                return self._tcp_teardown_session_id
            return self._tcp_session_id

        session_id = self._tcp_attempt_session_id
        if session_id is None:
            session_id = self._tcp_session_id
        self._tcp_teardown_reason = reason
        self._tcp_teardown_detail = detail
        self._tcp_teardown_session_id = session_id
        self._tcp_session_id += 1
        return session_id

    def _cancel_tcp_connect(self):
        """Cancel a lookup/connection attempt without sending a rover stop."""
        if self._tcp_socket.state() not in (
                QAbstractSocket.HostLookupState,
                QAbstractSocket.ConnectingState):
            return
        session_id = self._begin_tcp_teardown(self.TCP_USER_CANCEL)
        self._tcp_connect_timer.stop()
        self._prepare_for_disconnect()
        self._tcp_rx_buffer.clear()
        self._tcp_socket.abort()
        self._finalize_tcp_disconnected(self.TCP_USER_CANCEL, session_id)

    def _on_tcp_connect_timeout(self):
        if (self._tcp_teardown_reason is not None or
                self._tcp_socket.state() not in (
                    QAbstractSocket.HostLookupState,
                    QAbstractSocket.ConnectingState)):
            return
        session_id = self._begin_tcp_teardown(self.TCP_CONNECT_TIMEOUT)
        self._tcp_connect_timer.stop()
        self._prepare_for_disconnect()
        self._tcp_rx_buffer.clear()
        self._tcp_socket.abort()
        self._finalize_tcp_disconnected(self.TCP_CONNECT_TIMEOUT, session_id)

    def _on_tcp_connected(self):
        attempt_id = self._tcp_attempt_session_id
        valid = (
            self._tcp_socket.state() == QAbstractSocket.ConnectedState
            and attempt_id is not None
            and attempt_id == self._tcp_session_id
            and self._tcp_teardown_reason is None
            and self._tcp_finalized_session_id != attempt_id
        )
        if not valid:
            if self._tcp_teardown_reason is None:
                session_id = self._begin_tcp_teardown(
                    self.TCP_STALE_CONNECT)
            else:
                session_id = self._tcp_teardown_session_id
            self._tcp_connect_timer.stop()
            self._prepare_for_disconnect()
            if self._tcp_socket.state() != QAbstractSocket.UnconnectedState:
                self._tcp_socket.abort()
            self._finalize_tcp_disconnected(
                self._tcp_teardown_reason or self.TCP_STALE_CONNECT,
                session_id)
            return

        self._tcp_connect_timer.stop()
        self._tcp_connected_session_id = attempt_id
        self.connected = True

        # Set socket options
        try:
            self._tcp_socket.setSocketOption(
                QAbstractSocket.LowDelayOption, 1)
        except Exception:
            pass
        try:
            self._tcp_socket.setSocketOption(
                QAbstractSocket.KeepAliveOption, 1)
        except Exception:
            pass

        host = self._tcp_socket.peerAddress().toString()
        port = self._tcp_socket.peerPort()
        self._lbl_status.setText("Status: CONNECTED")
        self._lbl_status.setStyleSheet(
            f"color: {self._colors()['success_bright']}; font-weight: bold;"
        )
        self._btn_connect.setText("Disconnect")
        self._style_connection_button()
        self._lbl_qs_port.setText(f"Link: {host}:{port}")
        self._lbl_qs_port.setStyleSheet(
            self._style_badge(self._colors()['accent_gold']))
        self._log_info(f"Connected to {host}:{port}")

        # Start heartbeat — send one immediately, then periodically
        self._h7_link_status = "UNKNOWN"
        self._update_h7_link_badge()
        self._send_heartbeat()
        self._heartbeat_timer.start()
        # Query link status several times after the first heartbeat.  Each
        # callback captures this connection generation and is invalidated by
        # disconnect/reconnect before it is allowed to write.
        sid = self._tcp_session_id
        self._start_linkstat_retries(sid)

        # Start freshness tracking for new session
        self._reset_freshness_timestamps()
        self._freshness_timer.start()

    def _on_tcp_disconnected(self):
        if self._tcp_teardown_reason is None:
            session_id = self._begin_tcp_teardown(self.TCP_REMOTE_CLOSE)
        else:
            session_id = self._tcp_teardown_session_id
        self._finalize_tcp_disconnected(
            self._tcp_teardown_reason or self.TCP_REMOTE_CLOSE,
            session_id)

    def _on_tcp_ready_read(self):
        data = self._tcp_socket.readAll()
        if not data:
            return
        self._tcp_rx_buffer.extend(data)

        # Guard against buffer overflow
        if len(self._tcp_rx_buffer) > MAX_TCP_RX_BUFFER_SIZE:
            detail = "TCP RX buffer overflow — aborting connection."
            session_id = self._begin_tcp_teardown(
                self.TCP_RX_OVERFLOW, detail)
            reason = self._tcp_teardown_reason or self.TCP_RX_OVERFLOW
            self._prepare_for_disconnect()
            self._tcp_rx_buffer.clear()
            self._tcp_socket.abort()
            self._finalize_tcp_disconnected(
                reason, session_id)
            return

        # Extract complete lines
        while b"\n" in self._tcp_rx_buffer:
            line_bytes, self._tcp_rx_buffer = self._tcp_rx_buffer.split(b"\n", 1)
            line_bytes = line_bytes.rstrip(b"\r")
            if not line_bytes:
                continue
            text = line_bytes.decode("utf-8", errors="replace")
            if text:
                self._on_rx_line(text)

    def _on_tcp_error(self, error):
        # Any error emitted by an already-classified local teardown is an
        # expected consequence of disconnectFromHost()/abort().
        if self._tcp_teardown_reason is not None:
            return

        if error == QAbstractSocket.RemoteHostClosedError:
            session_id = self._begin_tcp_teardown(self.TCP_REMOTE_CLOSE)
            self._prepare_for_disconnect()
            if self._tcp_socket.state() == QAbstractSocket.UnconnectedState:
                self._finalize_tcp_disconnected(
                    self.TCP_REMOTE_CLOSE, session_id)
            return

        detail = self._tcp_socket.errorString()
        session_id = self._begin_tcp_teardown(
            self.TCP_SOCKET_ERROR, detail)
        self._prepare_for_disconnect()
        self._tcp_rx_buffer.clear()
        self._tcp_socket.abort()
        self._finalize_tcp_disconnected(
            self.TCP_SOCKET_ERROR, session_id)

    def _on_tcp_state_changed(self, state):
        """Render connection controls from QTcpSocket state only."""
        if state == QAbstractSocket.UnconnectedState:
            text, enabled, fields_enabled = "Connect", True, True
        elif state in (QAbstractSocket.HostLookupState,
                       QAbstractSocket.ConnectingState):
            text, enabled, fields_enabled = "Cancel", True, False
        elif state == QAbstractSocket.ConnectedState:
            text, enabled, fields_enabled = "Disconnect", True, False
        else:  # ClosingState (and any future non-interactive state)
            text, enabled, fields_enabled = "Disconnecting...", False, False

        self._btn_connect.setText(text)
        self._btn_connect.setEnabled(enabled)
        self._host_edit.setEnabled(fields_enabled)
        self._port_spin.setEnabled(fields_enabled)
        self._style_connection_button()

    def _prepare_for_disconnect(self, *, stop_link_timers: bool = True):
        """Stop command producers once for the ending attempt/session.

        Controlled user/window disconnect may defer stopping heartbeat until
        after the best-effort stop has been queued.  Finalization calls this
        again safely and ensures link timers are stopped.
        """
        session_id = self._tcp_teardown_session_id
        if session_id is None:
            session_id = self._tcp_attempt_session_id
        if session_id is None:
            session_id = self._tcp_session_id

        if self._tcp_prepared_session_id != session_id:
            self._tcp_prepared_session_id = session_id
            self._reset_input_state(send_stop=False)
            self._pending_mode = None
            self._pending_mode_timer.stop()
            if (self._tuning_send_timer.isActive() or
                    self._tuning_send_queue):
                self._tuning_send_timer.stop()
                self._tuning_send_queue.clear()
                if self._tuning_dialog_ref is not None:
                    try:
                        self._tuning_dialog_ref._set_send_buttons_enabled(True)
                    except Exception:
                        pass
                    self._tuning_dialog_ref = None
            if self._cfgread_motor is not None:
                dlg = self._cfgread_dialog
                if dlg is not None:
                    try:
                        dlg._set_read_status(
                            self._cfgread_motor, "Disconnected", success=False)
                    except Exception:
                        pass
            self._cfgread_cleanup()
            self._arm_stop_polling()
            self._drill_stop_polling()
            self._freshness_timer.stop()

        if stop_link_timers:
            self._stop_link_timers()

    def _set_disconnected_ui(self):
        """Render disconnected badges; lifecycle cleanup lives elsewhere."""
        self._lbl_status.setText("Status: DISCONNECTED")
        self._lbl_status.setStyleSheet(
            f"color: {self._colors()['danger']}; font-weight: bold;"
        )
        self._lbl_qs_port.setText("Link: Disconnected")
        self._lbl_qs_port.setStyleSheet(
            self._style_badge(self._colors()['danger']))
        self._on_tcp_state_changed(self._tcp_socket.state())

    def _finalize_tcp_disconnected(
            self, reason: str, session_id: int | None = None) -> bool:
        """Finalize one attempt/session once and emit its one final log."""
        if session_id is None:
            session_id = self._tcp_teardown_session_id
        if session_id is None:
            session_id = self._tcp_attempt_session_id
        if session_id is None:
            session_id = self._tcp_session_id
        if self._tcp_finalized_session_id == session_id:
            return False

        self._tcp_finalized_session_id = session_id
        self._tcp_connect_timer.stop()
        self._prepare_for_disconnect()
        self._tcp_rx_buffer.clear()
        self.connected = False
        self._tcp_connected_session_id = None
        self._h7_link_status = "UNKNOWN"
        self._update_h7_link_badge()
        self._mark_all_freshness_disconnected()
        self._set_disconnected_ui()

        if not self._window_closing:
            if reason == self.TCP_USER_DISCONNECT:
                self._log_info("Disconnected by user.")
            elif reason == self.TCP_USER_CANCEL:
                self._log_info("Connection attempt cancelled.")
            elif reason == self.TCP_CONNECT_TIMEOUT:
                self._log_err("Connection timed out.")
            elif reason == self.TCP_SOCKET_ERROR:
                detail = self._tcp_teardown_detail or "Unknown socket error"
                self._log_err(f"TCP error: {detail}")
            elif reason == self.TCP_REMOTE_CLOSE:
                self._log_warn("Connection lost.")
            elif reason in (self.TCP_RX_OVERFLOW, self.TCP_TX_OVERFLOW):
                self._log_err(self._tcp_teardown_detail or "TCP overflow")
            # STALE_CONNECT and WINDOW_CLOSE are intentionally silent.

        self._tcp_attempt_session_id = None
        return True

    def _disconnect(self):
        """Request a controlled disconnect and let Qt finalize it."""
        if self._tcp_socket.state() != QAbstractSocket.ConnectedState:
            return
        session_id = self._begin_tcp_teardown(self.TCP_USER_DISCONNECT)
        self._prepare_for_disconnect(stop_link_timers=False)
        self._send_cmd("stop")
        self._stop_link_timers()
        self._tcp_connect_timer.stop()
        if self._tcp_socket.state() == QAbstractSocket.ConnectedState:
            self._tcp_socket.flush()
            self._tcp_socket.disconnectFromHost()
        if self._tcp_socket.state() == QAbstractSocket.UnconnectedState:
            self._finalize_tcp_disconnected(
                self.TCP_USER_DISCONNECT, session_id)

    # ======================================================================
    #  TCP Receive
    # ======================================================================

    def _update_h7_link_badge(self):
        """Update the H7 Control Link badge in Rover Status."""
        if not hasattr(self, "_lbl_qs_h7_link"):
            return
        c = self._colors()
        status = self._h7_link_status
        if status == "ALIVE":
            color = c['success_bright']
        elif status == "TIMEOUT":
            color = c['danger_bright']
        else:
            color = c['text_muted']
        self._lbl_qs_h7_link.setText(f"H7 Control Link: {status}")
        self._lbl_qs_h7_link.setStyleSheet(self._style_badge(color))

    def _query_linkstat(self):
        """Send a diagnostic-only linkstat query; this is not a keepalive."""
        if self._tcp_is_connected():
            self._send_cmd("linkstat", track_arm=False)

    def _stop_link_timers(self):
        """Stop the heartbeat and every pending link-status retry timer."""
        self._heartbeat_timer.stop()
        for timer in self._linkstat_retry_timers:
            timer.stop()
            if timer.property("tcp_linkstat_connected"):
                try:
                    timer.timeout.disconnect()
                except (RuntimeError, TypeError):
                    pass
                timer.setProperty("tcp_linkstat_connected", False)

    def _start_linkstat_retries(self, session_id: int):
        """Arm the configured session-guarded diagnostic status retries."""
        for timer in self._linkstat_retry_timers:
            timer.stop()
            if timer.property("tcp_linkstat_connected"):
                try:
                    timer.timeout.disconnect()
                except (RuntimeError, TypeError):
                    pass
                timer.setProperty("tcp_linkstat_connected", False)
            timer.timeout.connect(
                lambda sid=session_id: self._query_linkstat_for_session(sid))
            timer.setProperty("tcp_linkstat_connected", True)
            timer.start()

    def _query_linkstat_for_session(self, session_id: int):
        """Session-guarded linkstat query — dropped if session changed."""
        if self._is_current_tcp_session(session_id):
            self._send_cmd("linkstat", track_arm=False)

    def _parse_pc_link_line(self, line: str) -> bool:
        """Parse H7 PC-link watchdog log lines and update link-status badge.

        Recognized formats:
            [ERROR] [PC_LINK] TIMEOUT,AGE_MS:2000,ACTION:STOP_DISARM
            [INFO] [PC_LINK] RECOVERED
            PC_LINK,SEEN:1,ALIVE:1,TIMEOUT:0,AGE_MS:123,LIMIT_MS:2000
        """
        status = _parse_pc_link_status(line)
        if status is None:
            return False
        self._h7_link_status = status
        self._update_h7_link_badge()
        return True

    def _on_rx_line(self, line: str):
        # Motor telemetry is exclusive — a [TEL][FL] line must not also
        # update arm or drill tables.
        if self._parse_motor_telemetry_line(line):
            self._parse_uart_error_line(line)
            return

        # Link-lost / recovered is orthogonal state that may accompany
        # other content, so always run it.
        self._parse_rx_for_motor_state(line)
        self._parse_uart_error_line(line)

        # Operating-mode confirmation is exclusive.
        if self._parse_operating_mode_confirm(line):
            return

        # PC_LINK diagnostic is exclusive.
        if self._parse_pc_link_line(line):
            return

        # Exact H7 stream confirmations are state records, not telemetry.
        if self._parse_imu_stream_line(line):
            return

        # IMU / MAG telemetry is exclusive.
        if self._parse_imu_line(line):
            return

        # cfgcache parsing is exclusive.
        if self._parse_cfgcache_line(line):
            return

        # UART8 records are normalized once at the protocol boundary.  All H7
        # parsing above deliberately used the original raw logger line.
        arm_payload = _extract_arm_rx_payload(line)
        if arm_payload is None:
            return

        # Arm motor telemetry (M1-M3, M6) — may overlap with drill for M4/M5.
        # Both parsers are tolerant: they check motor tags and return False
        # for non-matching lines, so running both is safe.
        self._arm_parse_motors(arm_payload)
        self._arm_process_rx(arm_payload)

        # Drill telemetry parsers.
        self._parse_drill_get_motors_line(arm_payload)
        self._parse_drill_mode_line(arm_payload)
        self._parse_drill_fault_line(arm_payload)

    def _parse_rx_for_motor_state(self, line: str):
        """Update Link column if link-lost/recovered detected."""
        lower = line.lower()
        link_col = self.MOTOR_COL["link"]
        for tag, row in self.MOTOR_ROW.items():
            if f"link_lost][{tag}" in lower:
                self._motor_link_state[tag] = "LOST"
                item = self._motor_table.item(row, link_col)
                if item:
                    item.setText("LOST")
                    item.setForeground(QColor(self._colors()["danger"]))
            if f"link_recovered][{tag}" in lower:
                self._motor_link_state[tag] = "OK"
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
            item.setText(f"FC{fc}: {self._fault_name(fc)}")
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

    # ======================================================================
    #  Telemetry Freshness Tracking
    # ======================================================================

    def _set_imu_stream_state(self, state: str):
        """Store and render the main window's authoritative stream state."""
        self._imu_stream_state = state
        dlg = self._imu_settings_dialog
        if dlg is not None:
            dlg.update_stream_status(state)

    def _set_shared_imu_mag_expectation(self, expected: bool | None,
                                        *, restart: bool = False):
        """Set expectation for the H7 stream shared by MPU and MAG.

        The timestamps remain independent even though the command controlling
        their periodic production is shared.
        """
        now = time.monotonic()
        for key in ("IMU", "MAG"):
            if expected is True:
                if restart or self._telemetry_expected.get(key) is not True:
                    self._telemetry_expected_since[key] = now
            else:
                self._telemetry_expected_since[key] = 0.0
            self._telemetry_expected[key] = expected
        if restart:
            self._freshness_imu = 0.0
            self._freshness_mag = 0.0
        if expected is not True or restart:
            self._freshness_imu_stale = False
            self._freshness_mag_stale = False

    def _clear_sensor_one_shots(self):
        self._imu_one_shot_session_id = None
        self._imu_one_shot_deadline = 0.0
        self._mag_one_shot_session_id = None
        self._mag_one_shot_deadline = 0.0

    def _clear_imu_stream_detection(self):
        """Clear passive stream evidence from any TCP session."""
        self._imu_stream_detect_session_id = None
        self._imu_stream_detect_first_rx = 0.0
        self._imu_stream_detect_count = 0
        self._imu_stream_detect_suppress_session_id = None
        self._imu_stream_detect_suppress_until = 0.0

    def _expire_imu_stream_detection(self, now: float):
        """Discard an incomplete passive-detection sequence without logging."""
        if (self._imu_stream_detect_suppress_session_id is not None and
                (not self._is_current_tcp_session(
                    self._imu_stream_detect_suppress_session_id) or
                 now >= self._imu_stream_detect_suppress_until)):
            self._imu_stream_detect_suppress_session_id = None
            self._imu_stream_detect_suppress_until = 0.0
        if self._imu_stream_detect_count <= 0:
            return
        if (self._imu_stream_state != self.IMU_STREAM_UNKNOWN or
                not self._is_current_tcp_session(
                    self._imu_stream_detect_session_id) or
                (now - self._imu_stream_detect_first_rx) * 1000.0 >
                IMU_STREAM_AUTODETECT_MAX_GAP_MS):
            self._clear_imu_stream_detection()

    def _observe_imu_stream_candidate(self, now: float):
        """Use repeated unsolicited valid MPU_IMU records to detect streaming."""
        session_id = self._tcp_session_id
        if (not self._is_current_tcp_session(session_id) or
                self._imu_stream_state != self.IMU_STREAM_UNKNOWN or
                self._imu_one_shot_session_id is not None):
            return
        if (self._imu_stream_detect_suppress_session_id == session_id and
                now < self._imu_stream_detect_suppress_until):
            return
        if self._imu_stream_detect_suppress_session_id is not None:
            self._imu_stream_detect_suppress_session_id = None
            self._imu_stream_detect_suppress_until = 0.0

        gap_ms = ((now - self._imu_stream_detect_first_rx) * 1000.0
                  if self._imu_stream_detect_first_rx > 0.0 else 0.0)
        if (self._imu_stream_detect_session_id != session_id or
                self._imu_stream_detect_count <= 0 or
                gap_ms > IMU_STREAM_AUTODETECT_MAX_GAP_MS):
            self._imu_stream_detect_session_id = session_id
            self._imu_stream_detect_first_rx = now
            self._imu_stream_detect_count = 1
            return

        self._imu_stream_detect_count += 1
        if self._imu_stream_detect_count < IMU_STREAM_AUTODETECT_MIN_PACKETS:
            return

        first_rx = self._imu_stream_detect_first_rx
        self._clear_imu_stream_detection()
        self._telemetry_expected["IMU"] = True
        self._telemetry_expected["MAG"] = True
        self._telemetry_expected_since["IMU"] = first_rx
        self._telemetry_expected_since["MAG"] = (
            self._freshness_mag if self._freshness_mag > 0.0 else now)
        self._freshness_imu_stale = False
        self._freshness_mag_stale = False
        self._set_imu_stream_state(self.IMU_STREAM_ON)
        self._update_freshness_ui()
        self._log_info("Existing H7 IMU stream detected")

    def _reset_sensor_session_state(self, *, update_ui: bool = True):
        """Clear all session-local IMU/MAG intent and freshness state."""
        self._set_imu_stream_state(self.IMU_STREAM_UNKNOWN)
        self._set_shared_imu_mag_expectation(None)
        self._freshness_imu = 0.0
        self._freshness_mag = 0.0
        self._freshness_imu_stale = False
        self._freshness_mag_stale = False
        self._clear_sensor_one_shots()
        self._clear_imu_stream_detection()
        if update_ui:
            self._update_freshness_ui()

    def _request_imu_stream_on(self) -> bool:
        """Request the H7 shared MPU/MAG stream for the current TCP session."""
        session_id = self._tcp_session_id
        if not self._is_current_tcp_session(session_id):
            return False
        if not self._send_cmd("imu stream on"):
            return False
        if not self._is_current_tcp_session(session_id):
            return False

        self._clear_sensor_one_shots()
        self._clear_imu_stream_detection()
        self._set_imu_stream_state(self.IMU_STREAM_STARTING)
        self._set_shared_imu_mag_expectation(True, restart=True)
        self._update_freshness_ui()
        return True

    def _request_imu_stream_off(self) -> bool:
        """Request stream stop and make both sensors immediately IDLE."""
        session_id = self._tcp_session_id
        if not self._is_current_tcp_session(session_id):
            return False
        if not self._send_cmd("imu stream off"):
            return False
        if not self._is_current_tcp_session(session_id):
            return False

        self._clear_sensor_one_shots()
        self._clear_imu_stream_detection()
        self._set_imu_stream_state(self.IMU_STREAM_STOPPING)
        self._set_shared_imu_mag_expectation(False)
        self._update_freshness_ui()
        return True

    def _request_imu_one_shot(self, command: str) -> bool:
        """Send one of the H7's bounded one-shot MPU read commands."""
        if command not in ("mpuraw", "mpuconv"):
            return False
        session_id = self._tcp_session_id
        if not self._is_current_tcp_session(session_id):
            return False
        if not self._send_cmd(command):
            return False
        if not self._is_current_tcp_session(session_id):
            return False
        self._clear_imu_stream_detection()
        self._imu_one_shot_session_id = session_id
        self._imu_one_shot_deadline = (
            time.monotonic() + SENSOR_ONE_SHOT_TIMEOUT_MS / 1000.0)
        return True

    def _request_mag_one_shot(self, command: str) -> bool:
        """Send one of the H7's bounded one-shot magnetometer reads."""
        if command not in ("magraw", "magimu"):
            return False
        session_id = self._tcp_session_id
        if not self._is_current_tcp_session(session_id):
            return False
        if not self._send_cmd(command):
            return False
        if not self._is_current_tcp_session(session_id):
            return False
        self._mag_one_shot_session_id = session_id
        self._mag_one_shot_deadline = (
            time.monotonic() + SENSOR_ONE_SHOT_TIMEOUT_MS / 1000.0)
        return True

    def _expire_sensor_one_shots(self, now: float):
        """Expire pending one-shot response windows without creating STALE."""
        if (self._imu_one_shot_session_id is not None and
                (not self._is_current_tcp_session(
                    self._imu_one_shot_session_id) or
                 now >= self._imu_one_shot_deadline)):
            self._imu_one_shot_session_id = None
            self._imu_one_shot_deadline = 0.0
        if (self._mag_one_shot_session_id is not None and
                (not self._is_current_tcp_session(
                    self._mag_one_shot_session_id) or
                 now >= self._mag_one_shot_deadline)):
            self._mag_one_shot_session_id = None
            self._mag_one_shot_deadline = 0.0

    def _parse_imu_stream_line(self, line: str) -> bool:
        """Apply one anchored H7 ``IMU_STREAM`` confirmation record."""
        enabled = _parse_imu_stream_status(line)
        if enabled is None:
            return False
        self._clear_imu_stream_detection()
        if enabled:
            self._set_shared_imu_mag_expectation(True)
            self._freshness_imu_stale = False
            self._freshness_mag_stale = False
            self._set_imu_stream_state(self.IMU_STREAM_ON)
        else:
            self._set_shared_imu_mag_expectation(False)
            self._set_imu_stream_state(self.IMU_STREAM_OFF)
        self._update_freshness_ui()
        return True

    def _reset_freshness_timestamps(self):
        """Reset all freshness timestamps and expectation state for a new TCP session.

        Drive motors (FL/FR/RL/RR) are marked expected immediately because
        the F411s stream telemetry continuously while connected.
        """
        now = time.monotonic()
        for m in ("FL", "FR", "RL", "RR"):
            self._freshness_motor[m] = 0.0
            self._freshness_motor_stale[m] = False
            self._motor_link_state[m] = "UNKNOWN"
            self._telemetry_expected[m] = True
            self._telemetry_expected_since[m] = now
        self._reset_sensor_session_state(update_ui=False)
        self._freshness_arm = 0.0
        self._freshness_arm_stale = False
        self._telemetry_expected["ARM"] = self._arm_poll_motors_timer.isActive()
        self._telemetry_expected_since["ARM"] = now if self._telemetry_expected["ARM"] else 0.0
        self._freshness_drill = 0.0
        self._freshness_drill_stale = False
        self._telemetry_expected["DRILL"] = self._drill_poll_motors_timer.isActive()
        self._telemetry_expected_since["DRILL"] = now if self._telemetry_expected["DRILL"] else 0.0

    def _mark_all_freshness_disconnected(self):
        """Mark all subsystems as disconnected (called on TCP disconnect)."""
        for m in ("FL", "FR", "RL", "RR"):
            self._freshness_motor_stale[m] = False
            self._motor_link_state[m] = "UNKNOWN"
            self._telemetry_expected[m] = None
            self._telemetry_expected_since[m] = 0.0
        self._reset_sensor_session_state(update_ui=False)
        self._freshness_arm_stale = False
        self._telemetry_expected["ARM"] = False
        self._telemetry_expected_since["ARM"] = 0.0
        self._freshness_drill_stale = False
        self._telemetry_expected["DRILL"] = False
        self._telemetry_expected_since["DRILL"] = 0.0
        # Reset group box titles to remove stale suffix
        self._update_freshness_ui()

    def _touch_motor_freshness(self, motor: str):
        """Mark motor telemetry as fresh (called on valid telemetry parse).

        Drive-motor telemetry is expected whenever TCP is connected (the F411s
        stream continuously).  On first touch after a reconnect the expectation
        transitions from UNKNOWN to FRESH.
        """
        if motor in self._freshness_motor:
            self._freshness_motor[motor] = time.monotonic()
            if self._telemetry_expected.get(motor) is None:
                self._telemetry_expected[motor] = True
                self._telemetry_expected_since[motor] = time.monotonic()

    def _touch_imu_freshness(self, record_kind: str = "MPU_IMU"):
        now = time.monotonic()
        self._freshness_imu = now
        if self._imu_one_shot_session_id == self._tcp_session_id:
            # Satisfy the one-shot on its first valid record, preserving the
            # TCP-9.1 behavior.  A short session-bound guard also excludes
            # mpuconv's immediately following MPU_IMU companion record.
            self._imu_one_shot_session_id = None
            self._imu_one_shot_deadline = 0.0
            self._imu_stream_detect_suppress_session_id = self._tcp_session_id
            self._imu_stream_detect_suppress_until = (
                now + IMU_ONE_SHOT_AUTODETECT_GUARD_MS / 1000.0)
            return
        if self._imu_stream_state == self.IMU_STREAM_STARTING:
            self._clear_imu_stream_detection()
            self._set_imu_stream_state(self.IMU_STREAM_ON)
        elif record_kind == "MPU_IMU":
            self._observe_imu_stream_candidate(now)

    def _touch_mag_freshness(self):
        self._freshness_mag = time.monotonic()
        if self._mag_one_shot_session_id == self._tcp_session_id:
            self._mag_one_shot_session_id = None
            self._mag_one_shot_deadline = 0.0

    def _touch_arm_freshness(self):
        self._freshness_arm = time.monotonic()

    def _touch_drill_freshness(self):
        self._freshness_drill = time.monotonic()

    def _check_telemetry_freshness(self):
        """Periodic timer callback — check each subsystem for staleness.

        Drive motors: expected whenever TCP is connected (F411s stream continuously).
        IMU / MAG: expected only while the confirmed/requested shared stream is active.
        ARM / DRILL: expected only while polling is active.
        """
        now = time.monotonic()
        connected = self._tcp_is_connected()
        self._expire_sensor_one_shots(now)
        self._expire_imu_stream_detection(now)

        # Per-motor freshness — drive motors expected when connected
        for m in ("FL", "FR", "RL", "RR"):
            expected = self._telemetry_expected[m]
            ts = self._freshness_motor[m]
            was_stale = self._freshness_motor_stale[m]

            if not connected:
                self._freshness_motor_stale[m] = False
            elif expected is False:
                # Intentionally idle
                self._freshness_motor_stale[m] = False
            elif expected is None:
                # Never received yet — UNKNOWN, not stale
                self._freshness_motor_stale[m] = False
            else:
                # expected is True — determine age from last telemetry or
                # from expectation-start time if no telemetry has arrived yet.
                if ts > 0.0:
                    reference = ts
                else:
                    reference = self._telemetry_expected_since.get(m, 0.0)
                if reference <= 0.0:
                    # No reference available yet — UNKNOWN
                    self._freshness_motor_stale[m] = False
                elif (now - reference) * 1000 > MOTOR_TELEMETRY_STALE_MS:
                    self._freshness_motor_stale[m] = True
                    if not was_stale:
                        self._log_warn(f"{m} telemetry stale")
                else:
                    self._freshness_motor_stale[m] = False
                    if was_stale:
                        self._log_info(f"{m} telemetry recovered")

        # Optional subsystems (IMU, MAG, ARM, DRILL)
        for key, ts_attr, stale_attr, threshold, name in [
            ("IMU",   "_freshness_imu",   "_freshness_imu_stale",   IMU_TELEMETRY_STALE_MS,   "IMU"),
            ("MAG",   "_freshness_mag",   "_freshness_mag_stale",   MAG_TELEMETRY_STALE_MS,   "MAG"),
            ("ARM",   "_freshness_arm",   "_freshness_arm_stale",   ARM_TELEMETRY_STALE_MS,   "Arm"),
            ("DRILL", "_freshness_drill", "_freshness_drill_stale", DRILL_TELEMETRY_STALE_MS, "Drill"),
        ]:
            self._check_single_freshness(
                getattr(self, ts_attr), getattr(self, stale_attr),
                threshold, name, connected, stale_attr,
                self._telemetry_expected[key],
                self._telemetry_expected_since.get(key, 0.0),
            )

        self._update_freshness_ui()

    def _check_single_freshness(self, ts: float, was_stale: bool,
                                 threshold_ms: int, name: str,
                                 connected: bool, attr: str,
                                 expected: bool | None,
                                 expected_since: float = 0.0):
        """Check one subsystem's freshness and update its stale state."""
        if not connected:
            setattr(self, attr, False)
            return
        if expected is False:
            # Intentionally idle — not stale
            setattr(self, attr, False)
            return
        if expected is None:
            # Never received or unknown expectation — not stale, just unknown
            setattr(self, attr, False)
            return
        # expected is True — determine age from last telemetry or
        # from expectation-start time if no telemetry has arrived yet.
        if ts > 0.0:
            reference = ts
        else:
            reference = expected_since
        if reference <= 0.0:
            setattr(self, attr, False)
            return
        now = time.monotonic()
        if (now - reference) * 1000 > threshold_ms:
            setattr(self, attr, True)
            if not was_stale:
                self._log_warn(f"{name} telemetry stale")
        else:
            setattr(self, attr, False)
            if was_stale:
                self._log_info(f"{name} telemetry recovered")

    def _sensor_freshness_label(self, key: str) -> str:
        """Return the independently computed IMU or MAG UI state."""
        if not self._tcp_is_connected():
            return "DISCONNECTED"
        expected = self._telemetry_expected.get(key)
        if expected is False:
            return "IDLE"
        if expected is None:
            return "UNKNOWN"
        stale = (self._freshness_imu_stale if key == "IMU"
                 else self._freshness_mag_stale)
        if stale:
            return "STALE"
        timestamp = self._freshness_imu if key == "IMU" else self._freshness_mag
        if timestamp <= 0.0:
            return ("STARTING" if self._imu_stream_state == self.IMU_STREAM_STARTING
                    else "UNKNOWN")
        return "FRESH"

    def _update_freshness_ui(self):
        """Update UI to reflect current freshness state.

        Motor Link column renders a combined status:
          - STALE | LOST  (freshness stale AND real link lost)
          - STALE         (freshness stale, link OK or unknown)
          - LOST          (real link lost, freshness OK)
          - UNKNOWN       (connected but never received)
          - OK            (fresh, link OK)
        """
        c = self._colors()
        connected = self._tcp_is_connected()

        # Motor table — update Link column
        for m in ("FL", "FR", "RL", "RR"):
            row = self.MOTOR_ROW.get(m)
            if row is None:
                continue
            col = self.MOTOR_COL["link"]
            item = self._motor_table.item(row, col)
            if item is None:
                continue

            stale = self._freshness_motor_stale[m]
            link = self._motor_link_state.get(m, "UNKNOWN")

            if not connected:
                item.setText("—")
                item.setForeground(QColor(c["text_muted"]))
            elif stale and link == "LOST":
                item.setText("STALE | LOST")
                item.setForeground(QColor(c["danger"]))
            elif stale:
                item.setText("STALE")
                item.setForeground(QColor(c["warning"]))
            elif link == "LOST":
                item.setText("LOST")
                item.setForeground(QColor(c["danger"]))
            elif self._freshness_motor[m] == 0.0:
                item.setText("UNKNOWN")
                item.setForeground(QColor(c["text_muted"]))
            else:
                item.setText("OK")
                item.setForeground(QColor(c["success_bright"]))

        # IMU/MAG group box — the stream control is shared, but freshness is
        # shown independently because either sensor can stop reporting alone.
        if hasattr(self, "_imu_grp"):
            if connected:
                title = (
                    f"IMU / MAG — IMU: {self._sensor_freshness_label('IMU')} | "
                    f"MAG: {self._sensor_freshness_label('MAG')}"
                )
            else:
                title = "IMU / MAG — DISCONNECTED"
            self._imu_grp.setTitle(title)

        # Arm group box
        if hasattr(self, "_arm_grp"):
            title = "Manipulation Arm Telemetry"
            if connected:
                exp = self._telemetry_expected.get("ARM")
                if exp is False:
                    title = "Manipulation Arm Telemetry (IDLE)"
                elif self._freshness_arm_stale:
                    title = "Manipulation Arm Telemetry (STALE)"
                elif self._freshness_arm == 0.0 and exp:
                    title = "Manipulation Arm Telemetry (UNKNOWN)"
            self._arm_grp.setTitle(title)

        # Drill group box
        if hasattr(self, "_drill_grp"):
            title = "Drill Telemetry"
            if connected:
                exp = self._telemetry_expected.get("DRILL")
                if exp is False:
                    title = "Drill Telemetry (IDLE)"
                elif self._freshness_drill_stale:
                    title = "Drill Telemetry (STALE)"
                elif self._freshness_drill == 0.0 and exp:
                    title = "Drill Telemetry (UNKNOWN)"
            self._drill_grp.setTitle(title)

    # -- F411 Motor telemetry parsing --------------------------------------

    def _parse_motor_telemetry_line(self, line: str) -> bool:
        """Detect and parse F411 telemetry from [TEL][MOTOR] or legacy [UART_RX].

        Require the same core fields used by the H7 telemetry classifier and
        reject malformed numeric values before touching freshness or UI state.
        Returns True only for a valid telemetry record.
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
        required = ("RPM", "PWM_ACT", "RXB")
        if any(key not in tel for key in required):
            return False
        numeric_keys = {
            "RPM", "T", "D", "APP_PH", "SP", "BRAKE", "FC", "H",
            "PWM_SET", "PWM_ACT", "QDROP", "RXB",
        }
        if any(
            key in tel and re.fullmatch(r"-?\d+", tel[key]) is None
            for key in numeric_keys
        ):
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

        self._touch_motor_freshness(motor)
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
                fc_item.setText(f"FC{fc}: {self._fault_name(fc)}")
                fc_item.setForeground(QColor(c["danger"]))

        # Re-render Error column (UART error has priority over FC)
        self._render_motor_error(motor)

        # Link -> OK when telemetry received (valid telemetry proves
        # the H7 is currently receiving data from this motor controller).
        # This also handles implicit recovery: LOST -> OK on valid telemetry.
        self._motor_link_state[motor] = "OK"
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
        """Detect and parse MPU/MAG one-shot and periodic telemetry records.

        Supports compact (MPU_IMU), detailed (MPU_CONV_MILLI), and magnetometer (MAG_IMU) formats.
        Unknown extra fields are ignored.  Missing fields keep last value.
        Returns True when the line contained recognized IMU telemetry.
        """
        # Raw one-shot records do not have converted display units, but they
        # still satisfy their bounded response window when the read succeeded.
        raw_mpu_fields = _parse_kv_payload(line, "MPU_RAW,")
        if raw_mpu_fields is not None:
            if raw_mpu_fields.get("OK") == 1:
                self._touch_imu_freshness("MPU_RAW")
            return True

        raw_mag_fields = _parse_kv_payload(line, "MAG_RAW,")
        if raw_mag_fields is not None:
            if raw_mag_fields.get("OK") == 1:
                self._touch_mag_freshness()
            return True

        # Try MPU_IMU first
        fields = _parse_kv_payload(line, "MPU_IMU,")
        record_kind = "MPU_IMU"
        if fields is None:
            fields = _parse_kv_payload(line, "MPU_CONV_MILLI,")
            record_kind = "MPU_CONV_MILLI"
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
            if mag_fields.get("OK") != 1:
                return True
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

            self._touch_mag_freshness()
            return True

        if fields is None:
            return False
        if fields.get("OK") != 1:
            return True

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

        # Temperature: centi-Celsius -> Celsius, 1 decimal place -> IMU table
        if "TC" in fields:
            self._set_imu_cell(R["Temp"], 1, f"{fields['TC'] / 100.0:.1f} °C")

        self._touch_imu_freshness(record_kind)
        return True

    # ======================================================================
    #  cfgcache read / parse
    # ======================================================================

    _RE_CFG_VALID = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+valid=(\d+)\s+updates=(\d+)\s+age_ms=(\d+)"
    )
    _RE_CFG_PI = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+"
        r"Kp_m=(-?\d+)\s+Ki_m=(-?\d+)\s+Kp=(-?[\d.]+)\s+Ki=(-?[\d.]+)"
    )
    _RE_CFG_BASE = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+Base\s+"
        r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
    )
    _RE_CFG_BOOST = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+Boost\s+"
        r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+ms=(\d+)"
    )
    _RE_CFG_RAMP = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+Ramp\s+up=(\d+)\s+down=(\d+)"
    )
    _RE_CFG_KICK = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+Kick\s+(ON|OFF)\s+duty=(\d+)\s+ms=(\d+)"
    )
    _RE_CFG_TELPER = re.compile(
        r"^\[CFG\]\[(FL|FR|RL|RR)\]\s+TelPer=(\d+)"
    )

    def _normalize_cfg_line(self, line: str) -> str:
        """Strip optional [INFO] / [WARN] / [DEBUG] prefix so regex can
        match [CFG] at the start of the string.

        H7 Logger_Log prepends e.g. "[INFO] " before the payload:
            [INFO] [CFG][RL] valid=1 ...
        After normalization:
            [CFG][RL] valid=1 ...
        """
        idx = line.find("[CFG]")
        if idx >= 0:
            return line[idx:]
        return line

    def _parse_cfgcache_line(self, line: str):
        """Parse [CFG][MOTOR] lines from H7 cfgcache output.

        Accumulates valid/pi/base/boost/ramp/kick/telper per motor.
        Uses a short debounce timer so optional trailing lines (Ramp,
        Kick, TelPer) are captured before applying to the dialog.
        """
        line = self._normalize_cfg_line(line)

        # Only process lines that start with [CFG]
        if not line.startswith("[CFG]"):
            return

        m = self._RE_CFG_VALID.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["valid"] = int(m.group(2))
            self._cfgread_pending[motor]["updates"] = int(m.group(3))
            self._cfgread_debounce_restart(motor)
            return

        m = self._RE_CFG_PI.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["kp_m"] = int(m.group(2))
            self._cfgread_pending[motor]["ki_m"] = int(m.group(3))
            self._cfgread_pending[motor]["has_pi"] = True
            self._cfgread_debounce_restart(motor)
            return

        m = self._RE_CFG_BASE.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["base"] = [
                int(m.group(i)) for i in range(2, 10)
            ]
            self._cfgread_pending[motor]["has_base"] = True
            self._cfgread_debounce_restart(motor)
            return

        m = self._RE_CFG_BOOST.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["boost"] = [
                int(m.group(i)) for i in range(2, 10)
            ]
            self._cfgread_pending[motor]["boost_ms"] = int(m.group(10))
            self._cfgread_pending[motor]["has_boost"] = True
            self._cfgread_debounce_restart(motor)
            return

        m = self._RE_CFG_RAMP.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["ramp_up"] = int(m.group(2))
            self._cfgread_pending[motor]["ramp_down"] = int(m.group(3))
            self._cfgread_debounce_restart(motor)
            return

        m = self._RE_CFG_KICK.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["kick_enabled"] = m.group(2)
            self._cfgread_pending[motor]["kick_duty"] = int(m.group(3))
            self._cfgread_pending[motor]["kick_ms"] = int(m.group(4))
            self._cfgread_debounce_restart(motor)
            return

        m = self._RE_CFG_TELPER.match(line)
        if m:
            motor = m.group(1)
            self._cfgread_pending.setdefault(motor, {})
            self._cfgread_pending[motor]["telper"] = int(m.group(2))
            self._cfgread_debounce_restart(motor)
            return

    def _cfgread_debounce_restart(self, motor: str):
        """Restart the debounce timer.  Called on every [CFG] line so
        optional trailing lines (Ramp, Kick, TelPer) are accumulated
        before the final apply."""
        if motor != self._cfgread_motor:
            return
        cfg = self._cfgread_pending.get(motor, {})
        # Only start debounce once the required parts are present
        if (cfg.get("valid") == 1
                and cfg.get("has_pi")
                and cfg.get("has_base")
                and cfg.get("has_boost")):
            self._cfgread_apply_timer.start()

    def _cfgread_apply_now(self):
        """Debounce timer fired — apply accumulated config to dialog."""
        motor = self._cfgread_motor
        if motor is None:
            return
        cfg = self._cfgread_pending.get(motor, {})
        if not (cfg.get("valid") == 1
                and cfg.get("has_pi")
                and cfg.get("has_base")
                and cfg.get("has_boost")):
            return

        # Stop all timers
        self._cfgread_apply_timer.stop()
        self._cfgread_retry_timer.stop()
        self._cfgread_timeout_timer.stop()
        dlg = self._cfgread_dialog
        self._cfgread_cleanup()

        # Store a copy in the persistent cache
        self._last_f411_cfg_by_motor[motor] = dict(cfg)
        self._last_f411_cfg_motor = motor

        if dlg is not None:
            dlg.apply_f411_tuning_config(motor, cfg)
            dlg._set_read_status(motor, f"Loaded {motor} config", success=True)

    def cfgread_start(self, motor: str, dialog):
        """Initiate a cfgread sequence for `motor`.

        Sends ``cfgread <MOTOR>`` immediately, then polls ``cfgcache <MOTOR>``
        after a short delay.  Retries up to 4 times (total ~2 s).
        """
        if not self._tcp_is_connected():
            self._log_warn(
                f"[CFGREAD] Not started (not connected): {motor}")
            if dialog is not None:
                dialog._set_read_status(
                    motor, "Not connected", success=False)
            return
        if self._cfgread_motor is not None:
            self._log_warn(f"[CFGREAD] Read already in progress for {self._cfgread_motor}")
            return

        self._cfgread_motor = motor
        self._cfgread_dialog = dialog
        self._cfgread_pending = {}
        self._cfgread_retry_count = 0

        # Send cfgread to trigger F411 cfg response
        self._send_cmd(f"cfgread {motor}")
        self._log_info(f"[CFGREAD] Sent cfgread {motor}")

        # After 500 ms, request cfgcache
        sid = self._tcp_session_id
        QTimer.singleShot(500, lambda sid=sid: self._cfgread_first_fetch_for_session(sid))

        # Start timeout
        self._cfgread_timeout_timer.start()

    def _cfgread_first_fetch(self):
        """First cfgcache request after initial delay."""
        if self._cfgread_motor is None:
            return
        self._send_cmd(f"cfgcache {self._cfgread_motor}")
        self._cfgread_retry_count = 1
        # Start retry timer if not yet complete
        if self._cfgread_motor is not None:
            self._cfgread_retry_timer.start()

    def _cfgread_first_fetch_for_session(self, session_id: int):
        """Session-guarded first cfgcache fetch — dropped if session changed."""
        if not self._is_current_tcp_session(session_id):
            return
        self._cfgread_first_fetch()

    def _cfgread_retry_fetch(self):
        """Retry cfgcache if not yet complete."""
        if self._cfgread_motor is None:
            self._cfgread_retry_timer.stop()
            return
        self._cfgread_retry_count += 1
        if self._cfgread_retry_count > 5:
            self._cfgread_retry_timer.stop()
            return
        self._send_cmd(f"cfgcache {self._cfgread_motor}")

    def _cfgread_on_timeout(self):
        """Called when cfgread timeout expires."""
        motor = self._cfgread_motor
        dlg = self._cfgread_dialog
        self._cfgread_retry_timer.stop()
        if motor is not None and dlg is not None:
            dlg._set_read_status(motor, f"Timeout reading {motor}", success=False)
            self._log_warn(f"[CFGREAD] Timeout reading {motor}")
        self._cfgread_cleanup()

    def _cfgread_cleanup(self):
        """Reset cfgread state."""
        self._cfgread_motor = None
        self._cfgread_dialog = None
        self._cfgread_pending = {}
        self._cfgread_retry_count = 0
        self._cfgread_retry_timer.stop()
        self._cfgread_timeout_timer.stop()
        self._cfgread_apply_timer.stop()

    def _send_cmd(self, cmd: str, *, quiet: bool = False, track_arm: bool = True) -> bool:
        """Send a raw command string to the H7 via TCP.

        quiet=True suppresses console logging (used for heartbeat).
        track_arm=False skips arm-command tracking (used for heartbeat).
        """
        if not self._tcp_is_connected():
            return False

        normalized = cmd.rstrip("\r\n")
        if not normalized:
            return False

        payload = (normalized + "\r\n").encode("utf-8")

        # TX backlog guard — prevent indefinite accumulation on slow Wi-Fi.
        # Check the projected queue size (current pending + new payload).
        pending = int(self._tcp_socket.bytesToWrite())
        projected = pending + len(payload)
        if projected > MAX_TCP_TX_BACKLOG:
            detail = (
                f"TCP TX backlog {pending} + {len(payload)} = {projected} "
                f"bytes exceeds limit {MAX_TCP_TX_BACKLOG} — aborting."
            )
            session_id = self._begin_tcp_teardown(
                self.TCP_TX_OVERFLOW, detail)
            reason = self._tcp_teardown_reason or self.TCP_TX_OVERFLOW
            self._prepare_for_disconnect()
            self._tcp_socket.abort()
            self._finalize_tcp_disconnected(
                reason, session_id)
            return False

        written = self._tcp_socket.write(payload)
        if written < 0:
            if not quiet:
                self._log_err(
                    f"TCP write failed: {self._tcp_socket.errorString()}"
                )
            return False

        if written != len(payload):
            if not quiet:
                self._log_warn(
                    f"TCP partial queue write: {written}/{len(payload)} bytes"
                )
            return False

        if track_arm:
            self._arm_track_command(normalized)
        return True

    def _send_manipulation_cmd(
            self, payload: str, *, quiet: bool = False,
            track_arm: bool = True) -> bool:
        """Send one F401 payload through H7's required ``arm`` route.

        Tracking receives the unprefixed F401 payload, while the TCP wire
        receives exactly one ``arm `` prefix.  `_send_cmd()` remains the sole
        owner of connection checks, backlog protection, CRLF framing and TCP
        writes.
        """
        normalized = _normalize_manipulation_payload(payload)
        if normalized is None:
            return False
        wire_command = _format_manipulation_command(normalized)
        if wire_command is None:
            return False
        sent = self._send_cmd(
            wire_command, quiet=quiet, track_arm=False)
        if sent and track_arm:
            self._arm_track_command(normalized)
        return sent

    def _send_heartbeat(self):
        """Send a heartbeat keepalive to the H7 (quiet, no arm tracking)."""
        self._send_cmd("hb", quiet=True, track_arm=False)

    def _arm_track_command(self, cmd: str):
        """Track arm joint commands for target angle updates."""
        parts = cmd.strip().split()
        if not parts:
            return
        keyword = parts[0].lower()
        if keyword == "goto" and len(parts) >= 3:
            joint = parts[1].upper()
            try:
                angle = float(parts[2])
                if joint in self.ARM_JOINTS and self.ARM_HAS_TARGET.get(joint):
                    self._arm_track_goto(joint, angle)
            except ValueError:
                pass
        elif keyword in ("rotate", "move") and len(parts) >= 3:
            joint = parts[1].upper()
            try:
                angle = float(parts[2])
                if joint in self.ARM_JOINTS and self.ARM_HAS_TARGET.get(joint):
                    current = self._arm_state[joint].get("degree")
                    if current and current != "—":
                        try:
                            cur_val = float(current.replace("°", ""))
                            new_tgt = cur_val + angle
                            self._arm_track_goto(joint, new_tgt)
                        except ValueError:
                            pass
            except ValueError:
                pass
        elif keyword == "stop":
            if len(parts) >= 2:
                joint = parts[1].upper()
                if joint in self.ARM_JOINTS:
                    self._arm_track_stop(joint)
            else:
                self._arm_track_stop(None)

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

    def _on_turn_ratio_spin_changed(self, value: float):
        """Sync self.turn_ratio when the spinbox value changes."""
        self.turn_ratio = round(value, 2)

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

    MOVEMENT_KEYS = ("W", "S", "A", "D", "T", "Y", "G", "H")

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
        if text == "Q":
            return "Q"
        if text == "E":
            return "E"
        if key == Qt.Key_T:
            return "T"
        if key == Qt.Key_Y:
            return "Y"
        if key == Qt.Key_G:
            return "G"
        if key == Qt.Key_H:
            return "H"
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
            if key_id in ("T", "Y", "G", "H"):
                self._send_cmd(self._arc_turn_cmd())
                self._update_motion_indicator(self._active_move_key)
                if not self._arc_repeat_timer.isActive():
                    self._arc_repeat_timer.start()
            else:
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
        elif key_id in ("Q", "E"):
            delta = -0.05 if key_id == "Q" else 0.05
            self.turn_ratio = max(0.0, min(1.0, self.turn_ratio + delta))
            self.turn_ratio = round(self.turn_ratio, 2)
            self._spin_turn_ratio.setValue(self.turn_ratio)
            self._log_info(f"Turn Ratio set to {self.turn_ratio:.2f}")

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
            if key_id in ("T", "Y", "G", "H"):
                self._arc_repeat_timer.stop()
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
        self._arc_repeat_timer.stop()
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

        if self._active_move_key and self._active_move_key in ("W", "S", "A", "D"):
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

    def _arc_turn_cmd(self) -> str:
        """Return the arc-turn command string for T/Y/G/H using current turn ratio."""
        motor_map = {"T": "fl", "Y": "fr", "G": "bl", "H": "br"}
        motor = motor_map.get(self._active_move_key, "fl")
        if self.mode == "RPM":
            return f"drive rpm {self.fb_rpm} {motor} tr {self.turn_ratio:.2f}"
        else:
            return f"drive duty {self.fb_pwm} {motor} tr {self.turn_ratio:.2f}"

    def _repeat_arc_turn(self):
        """Called every 500 ms by the arc-repeat timer for T/Y/G/H."""
        if self._active_move_key in ("T", "Y", "G", "H"):
            self._send_cmd(self._arc_turn_cmd())

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
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>Q / E</b></td><td>Turn Ratio -/+</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>T / Y</b></td><td>Arc Fwd-L / Fwd-R</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>G / H</b></td><td>Arc Bk-L / Bk-R</td>"
            f"<td></td><td></td></tr>"
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

        combo_html = (
            f"<table style='font-size:13px; color:{c['text']};' cellspacing='4'>"
            f"<tr><td style='color:{c['accent_gold']};'><b>Combo arc-turn:</b></td>"
            f"<td>W+A forward-left, W+D forward-right, S+A backward-left, S+D backward-right</td></tr>"
            f"</table>"
        )
        combo_label = QLabel(combo_html)
        combo_label.setTextFormat(Qt.RichText)
        layout.addWidget(combo_label)

        line3 = QFrame()
        line3.setFrameShape(QFrame.HLine)
        line3.setStyleSheet(f"color: {c['border']};")
        layout.addWidget(line3)

        console_html = (
            f"<table style='font-size:13px; color:{c['text']};' cellspacing='4'>"
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
        forwarded through _send_cmd() - the same TCP path used by all
        other H7 terminal commands.  If not connected, the command is still
        logged with a warning so the operator can see what would have been
        sent.
        with a warning so the operator can see what would have been sent.
        """
        if not self._tcp_is_connected():
            self._log_warn(
                f"[F411-TUNE] Not sent (not connected): {command}"
            )
            return
        self._log_info(f"[F411-TUNE] {command}")
        self._send_cmd(command)

    def _open_motor_settings(self):
        """Open the F411 Motor Tuning Settings dialog (Modal)."""
        dlg = MotorSettingsDialog(self, self)

        # Apply the most recently loaded motor config so the dialog
        # does not always reopen with hardcoded defaults.
        if self._last_f411_cfg_motor and self._last_f411_cfg_by_motor:
            m = self._last_f411_cfg_motor
            cached = self._last_f411_cfg_by_motor.get(m)
            if cached:
                dlg.apply_f411_tuning_config(m, cached)

        dlg.exec()

    def _open_fault_codes_dialog(self):
        """Open the Motor Fault Codes reference dialog."""
        c = self._colors()
        dlg = QDialog(self)
        dlg.setWindowTitle("Motor Fault Codes")
        dlg.setMinimumSize(720, 480)
        dlg.setStyleSheet(f"""
            QDialog {{
                background-color: {c['bg_main']};
                color: {c['text']};
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
            QTableWidget {{
                background-color: {c['bg_table']};
                border: 1px solid {c['border']};
                gridline-color: {c['gridline']};
                color: {c['text']};
                font-size: 12px;
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
        """)

        layout = QVBoxLayout(dlg)
        layout.setSpacing(10)

        title = QLabel("Motor Fault Codes")
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {c['accent_gold']};")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        subtitle = QLabel("F411 motor driver fault code reference")
        subtitle.setStyleSheet(f"font-size: 12px; color: {c['text_muted']};")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)

        table = QTableWidget(len(self.FAULT_CODES), 4)
        table.setHorizontalHeaderLabels(["Code", "Name", "Meaning", "Check / Possible Cause"])
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        table.setWordWrap(True)
        table.verticalHeader().setVisible(False)

        headers = table.horizontalHeader()
        if headers:
            headers.setSectionResizeMode(0, QHeaderView.Fixed)
            headers.setSectionResizeMode(1, QHeaderView.Fixed)
            headers.setSectionResizeMode(2, QHeaderView.Stretch)
            headers.setSectionResizeMode(3, QHeaderView.Stretch)
            headers.resizeSection(0, 50)
            headers.resizeSection(1, 120)

        for row, (code, name, meaning, check) in enumerate(self.FAULT_CODES):
            table.setItem(row, 0, QTableWidgetItem(str(code)))
            table.setItem(row, 1, QTableWidgetItem(name))
            table.setItem(row, 2, QTableWidgetItem(meaning))
            table.setItem(row, 3, QTableWidgetItem(check))

        layout.addWidget(table, 1)

        btn_close = QPushButton("Close")
        btn_close.setFixedWidth(120)
        btn_close.clicked.connect(dlg.accept)
        layout.addWidget(btn_close, alignment=Qt.AlignCenter)

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
        self._imu_settings_dialog.update_stream_status(
            self._imu_stream_state)
        self._imu_settings_dialog.show()

    # -- Manipulation Arm Settings dialog (non-modal) ---------------------

    def _open_arm_settings(self):
        """Open the Manipulation Arm Settings dialog (non-modal).

        Reuses an existing instance if still open.  Newly created dialogs
        are pre-populated from the most recently parsed ``params`` cache so
        the operator does not see blank fields after a refresh.
        """
        if self._arm_settings_dialog is not None and self._arm_settings_dialog.isVisible():
            self._arm_settings_dialog.raise_()
            self._arm_settings_dialog.activateWindow()
            return
        self._arm_settings_dialog = ManipulationArmSettingsDialog(self, self)
        # Re-apply the latest cached F401 params so the dialog sees live
        # values even if it was opened after a params refresh.
        if self._arm_params_cache:
            self._arm_settings_dialog.apply_params(self._arm_params_cache)
        self._arm_settings_dialog.show()

    def send_arm_setting_command(self, command: str):
        """Send one manipulation arm F401 setting command through H7 UART8.

        If not connected, _send_cmd() logs a warning and
        no exception is raised.  The command is also logged under the GUI
        console so the operator can see what would have been sent.
        """
        if not self._tcp_is_connected():
            self._log_warn(
                f"[ARM-SET] Not sent (not connected): {command}")
            return
        self._log_info(f"[ARM-SET] {command}")
        self._send_manipulation_cmd(command)

    def send_arm_setting_sequence(self, commands: list[str], interval_ms: int = 100):
        """Send a paced sequence through the centralized manipulation route.

        Commands are spaced by ``interval_ms`` (default 100) via QTimer so the
        H7 terminal / F4011 UART RX FIFO can absorb them.  If the serial
        port disconnects mid-sequence, remaining commands are logged as
        warnings instead of being sent.
        """
        if not commands:
            return
        if not self._tcp_is_connected():
            for cmd in commands:
                self._log_warn(
                    f"[ARM-SET] Not sent (not connected): {cmd}")
            return
        self._log_info(f"[ARM-SET] Sending {len(commands)} command(s)")
        # Send the first command immediately so single-row Sends feel snappy;
        # subsequent commands are paced by a one-shot QTimer chain.
        first = commands[0]
        self._log_info(f"[ARM-SET] {first}")
        self._send_manipulation_cmd(first)
        rest = commands[1:]
        if not rest:
            return

        sid = self._tcp_session_id

        def _send_next(idx: int):
            if idx >= len(rest):
                return
            if not self._is_current_tcp_session(sid):
                self._log_warn(
                    f"[ARM-SET] Sequence aborted (session changed)")
                return
            cmd = rest[idx]
            self._log_info(f"[ARM-SET] {cmd}")
            self._send_manipulation_cmd(cmd)
            QTimer.singleShot(interval_ms, lambda: _send_next(idx + 1))

        QTimer.singleShot(interval_ms, lambda: _send_next(0))

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
        if not self._tcp_is_connected():
            self._log_warn(f"[F411-TUNE] Not sent (not connected): {cmd}")
            # Drain remaining queue so buttons get re-enabled.
            self._tuning_send_queue.clear()
            self._tuning_send_timer.stop()
            self._log_warn("[F411-TUNE] Sequence aborted (not connected)")
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
        self._window_closing = True
        state = self._tcp_socket.state()

        if state == QAbstractSocket.ConnectedState:
            session_id = self._begin_tcp_teardown(self.TCP_WINDOW_CLOSE)
            self._prepare_for_disconnect(stop_link_timers=False)
            self._send_cmd("stop")
            self._stop_link_timers()
            if self._tcp_socket.state() == QAbstractSocket.ConnectedState:
                self._tcp_socket.flush()
                self._tcp_socket.disconnectFromHost()
            self._finalize_tcp_disconnected(
                self.TCP_WINDOW_CLOSE, session_id)
        elif state in (QAbstractSocket.HostLookupState,
                       QAbstractSocket.ConnectingState):
            session_id = self._begin_tcp_teardown(self.TCP_WINDOW_CLOSE)
            self._prepare_for_disconnect()
            self._tcp_socket.abort()
            self._finalize_tcp_disconnected(
                self.TCP_WINDOW_CLOSE, session_id)
        elif state == QAbstractSocket.ClosingState:
            if self._tcp_teardown_reason is None:
                session_id = self._begin_tcp_teardown(self.TCP_WINDOW_CLOSE)
            else:
                session_id = self._tcp_teardown_session_id
            self._finalize_tcp_disconnected(
                self._tcp_teardown_reason or self.TCP_WINDOW_CLOSE,
                session_id)
        else:
            self._tcp_teardown_reason = self.TCP_WINDOW_CLOSE

        self._tcp_connect_timer.stop()
        self._tcp_rx_buffer.clear()
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
