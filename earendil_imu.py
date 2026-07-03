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


# ════════════════════════════════════════════════════════════════════════════
#  Background Logo Watermark
#  Paints a low-opacity centered logo behind the GUI content in paintEvent().
# ════════════════════════════════════════════════════════════════════════════

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
#  H7 UART Error Log Parsing
#  Matches firmware output from Core/Src/motor_uart_dma.c:
#    [ERROR] <UART> UART error code: 0x00000004
#    [ERROR] <UART> UART error still unresolved: 0x00000004
#    [ERROR] <UART> error: <CODE> - <Description>
#    [INFO]  <UART> RX recovered after UART error
# ════════════════════════════════════════════════════════════════════════════
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

# ── Operating mode confirmation from H7 firmware (command_handler.c) ──────
#   The firmware logger (logger.c) prepends a level tag to every line, so
#   the actual serial output looks like:
#       [INFO] [MODE] DISARM active, motion commands locked
#       [INFO] [MODE] MANUAL active
#       [INFO] [MODE] AUTONOMOUS active
# This is the single source of truth for the GUI Operating Mode indicator.
_RE_OP_MODE_CONFIRM = re.compile(
    r"\[MODE\]\s+(DISARM|MANUAL|AUTONOMOUS)\s+active\b"
)

_RE_IMU_DATA = re.compile(
    r"^\[IMU\]\s+Roll:\s+([-\d.]+)\s+Pitch:\s+([-\d.]+)\s+Yaw:\s+([-\d.]+)$"
)

MPU_IMU_FIELDS = ("AX", "AY", "AZ", "GX", "GY", "GZ", "TC", "OK")


def parse_mpu_imu_line(line: str) -> dict | None:
    """Parse an MPU_IMU compact converted telemetry line.

    Expected format (may have prefixes like ``[RX-H7] [INFO]``):
        MPU_IMU,AX:-117,AY:-243,AZ:-981,GX:1313,GY:3855,GZ:2023,TC:2867,OK:1

    Returns a dict of int values, or *None* if the line is not valid.
    """
    if "MPU_IMU," not in line:
        return None
    try:
        vals: dict[str, int] = {}
        for part in line.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                k = k.strip()
                v = v.strip()
                if k in MPU_IMU_FIELDS:
                    vals[k] = int(v)
        if "AX" not in vals:
            return None
        return vals
    except (ValueError, KeyError):
        return None


# ════════════════════════════════════════════════════════════════════════════
#  Main GUI
# ════════════════════════════════════════════════════════════════════════════

class EarendilControlGui(QMainWindow):
    """
    Main rover control window.
    Left side: control panels.  Right side: serial console.
    """

    # ── Theme palettes ───────────────────────────────────────────────────────
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
            padding-top: 14px;
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

    # ── Constants ──────────────────────────────────────────────────────────
    REPEAT_INTERVAL_MS = 500
    # How long the GUI waits for an H7 confirmation of an operating-mode
    # command before warning that the mode change was not confirmed.
    OP_MODE_CONFIRM_TIMEOUT_MS = 3000
    DEFAULT_RPM = 100
    DEFAULT_PWM = 100
    RPM_MAX = 200
    PWM_MAX = 255
    VALUE_STEP = 5

    # ── Motor table row index ─────────────────────────────────────────────
    MOTOR_ROW = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}

    # ── UART → motor mapping (must match H7 firmware) ─────────────────────
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

    # ── Operating mode (DISARM / MANUAL / AUTONOMOUS) ─────────────────────
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
    # Order of LEDs left→right: red, yellow, green
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
        self.setWindowTitle("Earendil — Rover Control")
        self.setMinimumSize(1100, 650)

        # ── State ──────────────────────────────────────────────────────────
        self.current_theme = "dark"            # "dark" or "light"

        self.ser: serial.Serial | None = None
        self.reader_thread: SerialReaderThread | None = None
        self.connected = False

        self.mode = "RPM"               # "RPM" or "DUTY"
        self.current_rpm = self.DEFAULT_RPM
        self.current_pwm = self.DEFAULT_PWM

        self._operating_mode = "disarm"          # confirmed mode (H7 is source of truth)
        self._pending_mode: str | None = None   # mode requested by user, awaiting H7 confirm

        self._active_move_key: str | None = None   # current movement key (W/A/S/D)
        self._move_held: set[str] = set()          # held movement keys
        self._move_order: deque[str] = deque()     # movement key press order
        self._active_modifier: str | None = None   # "Shift" or "Ctrl" if held
        self._keys_held: set[str] = set()          # ALL held keys (prevents duplicates)

        # ── Motor UART error tracking ─────────────────────────────────────
        # motor -> current text shown in the Error column (col 4)
        self._motor_error_text: dict[str, str] = {"FL": "", "FR": "", "RL": "", "RR": ""}
        # uart -> decoded error parts accumulated within one report cycle
        self._uart_report_decoded: dict[str, list[str]] = {}

        # ── Build UI ───────────────────────────────────────────────────────
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

        # ── Top row: Serial Connection (left) + Rover Status (right) ───
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(8)
        top_row.addWidget(self._build_connection_group())
        top_row.addWidget(self._build_rover_status_group(), 1)

        left_layout.addLayout(top_row)

        # ── Mode / Value  (left)  +  Operating Mode (right)  in one row ──
        mode_op_row = QHBoxLayout()
        mode_op_row.setContentsMargins(0, 0, 0, 0)
        mode_op_row.setSpacing(8)
        mode_op_row.addWidget(self._build_mode_value_group(), 1)
        mode_op_row.addWidget(self._build_operating_mode_group())
        left_layout.addLayout(mode_op_row)

        left_layout.addWidget(self._build_motor_table_group())
        left_layout.addWidget(self._build_imu_group())
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

        # ── Operating-mode confirmation timeout ───────────────────────────
        # Started when a mode button sends a command; if the H7 does not
        # reply with a `[MODE] ... active` line in time, we warn and keep
        # the previously confirmed mode (no optimistic UI change).
        self._pending_mode_timer = QTimer(self)
        self._pending_mode_timer.setSingleShot(True)
        self._pending_mode_timer.setInterval(self.OP_MODE_CONFIRM_TIMEOUT_MS)
        self._pending_mode_timer.timeout.connect(self._on_pending_mode_timeout)

        # ── MPU IMU "Last Update" age refresh (1 s) ───────────────────────
        self._mpu_age_timer = QTimer(self)
        self._mpu_age_timer.setInterval(1000)
        self._mpu_age_timer.timeout.connect(self._tick_mpu_imu_age)

        # Prevent buttons from stealing keyboard focus (Space must always reach keyPressEvent)
        for btn in self.findChildren(QPushButton):
            btn.setFocusPolicy(Qt.NoFocus)

        # H7 input handles its own keyboard events; main window handles the rest
        self._h7_input.installEventFilter(self)
        self.setFocusPolicy(Qt.StrongFocus)

        # Apply the active theme to all widgets (stylesheet + inline styles +
        # background logo).  Done last so all widgets exist before re-styling.
        self._apply_theme()

        self._refresh_ports()
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
        # Initial dark-theme default; _apply_theme() re-styles for the active theme.
        self._btn_connect.setStyleSheet("QPushButton { background-color: #1e6e3e; color: #C0C0C0; }")
        self._btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self._btn_connect)

        self._lbl_status = QLabel("● Disconnected")
        self._lbl_status.setStyleSheet("color: #B00020; font-weight: bold;")
        self._lbl_status.setFixedWidth(120)
        lay.addWidget(self._lbl_status)

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

        # Theme toggle button — only changes visual theme, never sends a
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

        # ── Left: three LEDs (red / yellow / green) ──────────────────────
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

        # ── Right: status box on top, three buttons below ────────────────
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
        lay = QVBoxLayout(grp)

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
        self._motor_table.setFocusPolicy(Qt.NoFocus)
        self._motor_table.setMaximumHeight(160)
        lay.addWidget(self._motor_table)
        return grp

    # ── 9-axis IMU placeholder ────────────────────────────────────────────
    #   3 accel (X/Y/Z) + 3 gyro (X/Y/Z) + 3 mag (X/Y/Z).
    #   Values shown as "--" until firmware sends real IMU data; the parser
    #   hook (_update_imu_values) is wired but a no-op until then.
    IMU_FIELDS = ("AX", "AY", "AZ", "GX", "GY", "GZ", "MX", "MY", "MZ")

    # Labels shown in the IMU panel, in display order.
    _MPU_IMU_LABELS = (
        ("AX",     "Accel X"),
        ("AY",     "Accel Y"),
        ("AZ",     "Accel Z"),
        ("GX",     "Gyro X"),
        ("GY",     "Gyro Y"),
        ("GZ",     "Gyro Z"),
        ("TC",     "Temp"),
        ("STATUS", "Status"),
        ("AGE",    "Last Update"),
    )

    def _build_imu_group(self) -> QGroupBox:
        grp = QGroupBox("IMU / MPU Converted")
        grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        outer = QVBoxLayout(grp)
        outer.setContentsMargins(8, 6, 8, 6)
        outer.setSpacing(6)

        # ── IMU panel (field/value grid) ────────────────────────────────
        grid = QTableWidget(len(self._MPU_IMU_LABELS), 2)
        grid.setHorizontalHeaderLabels(["Field", "Value"])
        grid.setVerticalHeaderLabels([])
        hdrs = grid.horizontalHeader()
        if hdrs:
            hdrs.setSectionResizeMode(0, QHeaderView.ResizeToContents)
            hdrs.setSectionResizeMode(1, QHeaderView.Stretch)
        grid.setEditTriggers(QTableWidget.NoEditTriggers)
        grid.setFocusPolicy(Qt.NoFocus)
        grid.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        grid.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        for row, (key, label) in enumerate(self._MPU_IMU_LABELS):
            grid.setItem(row, 0, QTableWidgetItem(label))
            grid.setItem(row, 1, QTableWidgetItem("--"))

        for row in range(len(self._MPU_IMU_LABELS)):
            grid.setRowHeight(row, 18)
        grid.setMaximumHeight(len(self._MPU_IMU_LABELS) * 18 + 30)
        outer.addWidget(grid)
        self._mpu_imu_table = grid

        # Request button
        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)
        self._btn_mpu_conv = QPushButton("Request IMU Conv")
        self._btn_mpu_conv.clicked.connect(lambda: self._send_cmd("mpuconv"))
        btn_row.addWidget(self._btn_mpu_conv)
        btn_row.addStretch()
        outer.addLayout(btn_row)

        # ── 9-axis IMU table (Accel / Gyro / Mag) ──────────────────────
        self._imu_table = QTableWidget(3, 4)
        self._imu_table.setHorizontalHeaderLabels(["Sensor", "X", "Y", "Z"])
        imu_hdrs = self._imu_table.horizontalHeader()
        if imu_hdrs:
            imu_hdrs.setSectionResizeMode(QHeaderView.Stretch)
        self._imu_table.setVerticalHeaderLabels([])
        sensors = [("Accel", "m/s²"), ("Gyro", "°/s"), ("Mag", "µT")]
        for row, (name, _unit) in enumerate(sensors):
            self._imu_table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 4):
                self._imu_table.setItem(row, col, QTableWidgetItem("--"))
        self._imu_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._imu_table.setFocusPolicy(Qt.NoFocus)
        self._imu_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._imu_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._imu_table.setMaximumHeight(100)
        outer.addWidget(self._imu_table)

        # Tracking state for "Last Update" display.
        self._mpu_imu_last_ts: float | None = None
        self._mpu_imu_data: dict[str, int] = {}

        return grp

    def _update_imu_values(self, values: dict[str, float]):
        """Update the 9-axis IMU table.

        `values` keys: AX, AY, AZ, GX, GY, GZ, MX, MY, MZ.
        """
        rows = {"A": 0, "G": 1, "M": 2}
        for field in self.IMU_FIELDS:
            if field not in values:
                continue
            row = rows[field[0]]
            col = {"X": 1, "Y": 2, "Z": 3}[field[1]]
            item = self._imu_table.item(row, col)
            if item is not None:
                item.setText(f"{values[field]:.2f}")

    def _update_mpu_imu_panel(self, data: dict[str, int]):
        """Refresh the IMU panel with freshly parsed MPU_IMU converted data."""
        self._mpu_imu_data = data
        self._mpu_imu_last_ts = time.monotonic()

        status_ok = data.get("OK", 0) == 1
        status_text = "OK" if status_ok else "FAIL"

        def _mg_to_g(mg: int) -> str:
            return f"{mg / 1000:.3f} g"

        def _mdps_to_dps(mdps: int) -> str:
            return f"{mdps / 1000:.3f} dps"

        def _cx100_to_c(cx100: int) -> str:
            return f"{cx100 / 100:.2f} °C"

        display = {
            "AX":     _mg_to_g(data.get("AX", 0)),
            "AY":     _mg_to_g(data.get("AY", 0)),
            "AZ":     _mg_to_g(data.get("AZ", 0)),
            "GX":     _mdps_to_dps(data.get("GX", 0)),
            "GY":     _mdps_to_dps(data.get("GY", 0)),
            "GZ":     _mdps_to_dps(data.get("GZ", 0)),
            "TC":     _cx100_to_c(data.get("TC", 0)),
            "STATUS": status_text,
            "AGE":    "0.0 s",
        }

        for row, (key, _label) in enumerate(self._MPU_IMU_LABELS):
            item = self._mpu_imu_table.item(row, 1)
            if item is None:
                continue
            item.setText(display.get(key, "--"))
            if key == "STATUS":
                c = self._colors()
                item.setForeground(
                    QColor(c['success_bright'] if status_ok else c['danger_bright'])
                )

    def _tick_mpu_imu_age(self):
        """Called on a 1 s timer to keep 'Last Update' fresh."""
        if self._mpu_imu_last_ts is None:
            return
        elapsed = time.monotonic() - self._mpu_imu_last_ts
        item = self._mpu_imu_table.item(8, 1)  # AGE row
        if item is not None:
            item.setText(f"{elapsed:.1f} s")

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

        # ── H7 Console ────────────────────────────────────────────────
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

        # ── GUI Console ───────────────────────────────────────────────
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

    # ══════════════════════════════════════════════════════════════════════
    #  Theme Management
    # ══════════════════════════════════════════════════════════════════════
    #
    #  Theme switching is purely visual.  _toggle_theme() flips self.current_theme,
    #  regenerates the stylesheet, restyles every theme-aware inline style and
    #  repaints dynamic widgets to match the current state — WITHOUT touching
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

        # ── Static (builder-set) widgets re-styled to the active theme ──
        self._style_connection_button()
        self._style_connection_status()
        self._style_console_widgets()
        self._style_help_button()
        if hasattr(self, "_lbl_op_mode_status"):
            self._update_operating_mode_ui(self._operating_mode)

        # ── Dynamic state-driven widgets re-rendered to the active theme ──
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

    # ── Theme-aware style string helpers ───────────────────────────────────

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
        """Style the ● Connected / ● Disconnected label."""
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

    def _style_mode_value_labels(self):
        """Re-style the Mode label + Value label for the active theme + mode."""
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
        self._lbl_value.setStyleSheet(
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

    # ══════════════════════════════════════════════════════════════════════
    #  Console Logging
    # ══════════════════════════════════════════════════════════════════════

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
            self._style_connection_status()
            self._btn_connect.setText("Disconnect")
            self._style_connection_button()
            self._port_combo.setEnabled(False)
            self._baud_edit.setEnabled(False)

            self._log_info(f"Connected to {port} @ {baud}")
            self._lbl_qs_port.setText(f"Port: {port}")
            self._lbl_qs_port.setStyleSheet(self._style_badge(self._colors()['accent_gold']))
            self._mpu_age_timer.start()
        except Exception as e:
            self._log_err(f"Connect failed: {e}")

    def _disconnect(self):
        self._stop_reader()
        self._close_serial()
        self._mpu_age_timer.stop()
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
        # A pending mode change can never be confirmed while disconnected.
        self._pending_mode = None
        if self._pending_mode_timer.isActive():
            self._pending_mode_timer.stop()
        self._lbl_status.setText("● Disconnected")
        self._style_connection_status()
        self._btn_connect.setText("Connect")
        self._style_connection_button()
        self._port_combo.setEnabled(True)
        self._baud_edit.setEnabled(True)
        self._lbl_qs_port.setText("Port: Disconnected")
        self._lbl_qs_port.setStyleSheet(self._style_badge(self._colors()['danger']))

    # ══════════════════════════════════════════════════════════════════════
    #  Serial Receive
    # ══════════════════════════════════════════════════════════════════════

    def _on_rx_line(self, line: str):
        self._log_rx(line)
        self._parse_rx_for_motor_state(line)
        self._parse_uart_error_line(line)
        self._parse_operating_mode_confirm(line)
        self._parse_imu_line(line)

    def _parse_imu_line(self, line: str):
        conv = parse_mpu_imu_line(line)
        if conv is not None:
            self._update_mpu_imu_panel(conv)
            # Also feed the 9-axis table with converted values.
            self._update_imu_values({
                "AX": conv.get("AX", 0) / 1000.0,
                "AY": conv.get("AY", 0) / 1000.0,
                "AZ": conv.get("AZ", 0) / 1000.0,
                "GX": conv.get("GX", 0) / 1000.0,
                "GY": conv.get("GY", 0) / 1000.0,
                "GZ": conv.get("GZ", 0) / 1000.0,
            })
            return

        # Legacy [IMU] Roll/Pitch/Yaw format (if ever used by firmware).
        match = _RE_IMU_DATA.match(line)
        if match:
            try:
                roll = float(match.group(1))
                pitch = float(match.group(2))
                yaw = float(match.group(3))
                self._update_imu_values({"AX": roll, "AY": pitch, "AZ": yaw})
            except ValueError:
                pass

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

    # ── UART error / recovery parsing ───────────────────────────────────────
    #
    # The H7 firmware (motor_uart_dma.c) reports UART errors over the
    # terminal link (USART3) as plain log lines.  These are NOT motor
    # protocol frames (ACK/STATUS/FAULT), so the existing table parser
    # ignored them and only the console showed them.  These methods detect
    # those log lines and route them into the motor table's Error column
    # using the firmware's UART→motor mapping.

    def _set_motor_error(self, motor: str, text: str, is_error: bool):
        """Write `text` into the Error column (col 4) of the motor row."""
        row = self.MOTOR_ROW.get(motor)
        if row is None:
            return
        self._motor_error_text[motor] = text
        item = self._motor_table.item(row, 4)
        if item is not None:
            item.setText(text)
            item.setForeground(QColor("#B00020") if is_error else QColor("#D4AF37"))

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

        # RX recovered after a previous UART error → clear the Error column.
        m = _RE_UART_RECOVERED.match(line)
        if m:
            uart = m.group(1)
            motor = self.UART_TO_MOTOR.get(uart)
            if motor is not None:
                self._uart_report_decoded.pop(uart, None)
                self._set_motor_error(motor, "OK", is_error=False)
            return True

        return False

    # ── Operating-mode confirmation parsing ─────────────────────────────────
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
            f"not confirmed by H7 — keeping current mode "
            f"({self.OPERATING_MODES.get(self._operating_mode, {}).get('label', self._operating_mode)})."
        )

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
            self._log_h7("[TX-H7]", f"{text}  (not sent - not connected)", self._colors()['warning'])
            self._log_warn("Cannot send to H7: serial port is not connected.")
        else:
            self._send_cmd(text)
        self.setFocus()  # return focus to main window so WASD works again

    # ══════════════════════════════════════════════════════════════════════
    #  Mode / Value Management
    # ══════════════════════════════════════════════════════════════════════

    def _get_current_value(self) -> int:
        return self.current_rpm if self.mode == "RPM" else self.current_pwm

    # ── Operating mode (DISARM / MANUAL / AUTONOMOUS) ─────────────────────
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
                f"Requested {cfg['label']} — waiting for H7 confirmation..."
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
            self._lbl_value_label.setText("RPM Value:")
            self._lbl_value.setText(str(self.current_rpm))
            self._send_cmd("m speed")
        else:
            self._lbl_value_label.setText("Duty Value:")
            self._lbl_value.setText(str(self.current_pwm))
            self._send_cmd("m duty")
        # Re-style the Mode + Value labels for the active theme + mode.
        self._style_mode_value_labels()
        self._log_info(f"Mode changed to {new_mode}")
        self._lbl_qs_mode.setText(f"Mode: {new_mode}")
        self._lbl_qs_mode.setStyleSheet(self._style_badge(self._colors()['accent_gold']))

    def _toggle_mode(self):
        self._set_mode("DUTY" if self.mode == "RPM" else "RPM")

    def _adjust_value(self, delta: int):
        if self.mode == "RPM":
            self.current_rpm = max(0, min(self.RPM_MAX, self.current_rpm + delta))
            self._lbl_value.setText(str(self.current_rpm))
            self._log_info(f"RPM value set to {self.current_rpm}")
        else:
            self.current_pwm = max(0, min(self.PWM_MAX, self.current_pwm + delta))
            self._lbl_value.setText(str(self.current_pwm))
            self._log_info(f"Duty value set to {self.current_pwm}")

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

        title = QLabel("Earendil — Rover Control GUI Help")
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
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>W</b></td><td>Forward</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>Space</b></td><td>Stop</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>S</b></td><td>Backward</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>X</b></td><td>Brake</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>A</b></td><td>Left</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>M</b></td><td>Toggle RPM/DUTY</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>D</b></td><td>Right</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>I</b></td><td>Identify</td></tr>"
            f"<tr><td style='color:{c['accent_gold_bright']};'><b>LShift</b></td><td>Value +5</td>"
            f"<td style='color:{c['accent_gold_bright']};'><b>LCtrl</b></td><td>Value -5</td></tr>"
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
            f"<td>W/S/A/D sends f/b/l/r&lt;number&gt;  (cmd: m speed)</td></tr>"
            f"<tr><td style='color:{c['accent_gold']};'><b>DUTY mode:</b></td>"
            f"<td>W/S/A/D sends fd/bd/ld/rd&lt;number&gt;  (cmd: m duty)</td></tr>"
            f"</table>"
            f"<br>"
            f"<span style='color:{c['text_muted']};'>Held key repeats every 500 ms</span>"
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

    # ══════════════════════════════════════════════════════════════════════
    #  Cleanup
    # ══════════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        self._repeat_timer.stop()
        self._mpu_age_timer.stop()
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
