#!/usr/bin/env python3
"""
tcp_uart_bridge.py — Raspberry Pi TCP-to-Serial Bridge for STM32H723 Rover

Transparently forwards TCP bytes from a control PC to an H7 serial port
and vice versa.  The bridge accepts a single active TCP client at a time
and manages serial-port ownership, reconnect, and disconnect-event
behaviour.

Third-party dependency (install on Raspberry Pi):
    python3 -m pip install pyserial

Usage (serial-device is REQUIRED):

    # Run this on the Raspberry Pi (the machine physically connected to H7):
    ls -l /dev/serial/by-id/

    # For a remote GUI, listen on the Pi network and restrict the client:
    python3 tcp_uart_bridge.py \\
      --host 0.0.0.0 \\
      --port 5000 \\
      --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_XXXX-if02 \\
      --baud 115200 \\
      --allow-client 192.168.50.10/32 \\
      --log-file /tmp/bridge.log

The GUI must connect to the Raspberry Pi LAN address (for example,
192.168.50.20), not 127.0.0.1.  The default loopback listener is only for
the intentional same-machine case where the GUI also runs on the Pi.

Validate configuration without starting workers:

    python3 tcp_uart_bridge.py \\
      --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_XXXX-if02 \\
      --check-config

Find the persistent serial device path on the Raspberry Pi:
    ls -l /dev/serial/by-id/
"""

from __future__ import annotations

import argparse
import ipaddress
import logging
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial  # pyserial

# ---------------------------------------------------------------------------
# Default configuration constants
# ---------------------------------------------------------------------------

DEFAULT_LISTEN_HOST: str = "127.0.0.1"
DEFAULT_LISTEN_PORT: int = 5000

DEFAULT_BAUD_RATE: int = 115200
DEFAULT_SERIAL_READ_TIMEOUT: float = 0.05   # seconds
DEFAULT_SERIAL_WRITE_TIMEOUT: float = 0.2   # seconds
DEFAULT_SERIAL_RECONNECT_INTERVAL: float = 1.0  # seconds

DEFAULT_SEND_DISCONNECT_EVENT: bool = True
DEFAULT_DISCONNECT_COMMAND: str = "pc_disconnect\r\n"

DEFAULT_TCP_NODELAY: bool = True
DEFAULT_TCP_KEEPALIVE: bool = True
DEFAULT_LOG_LEVEL: str = "INFO"

# Read chunk size for both TCP and serial receivers.
# Not a protocol framing unit — just a sensible upper bound for recv/read.
_RECV_CHUNK_SIZE: int = 4096

# Bound for thread join timeouts during shutdown (seconds).
_JOIN_TIMEOUT: float = 3.0

# Valid log-level names accepted on the command line.
_VALID_LOG_LEVELS: tuple[str, ...] = (
    "DEBUG",
    "INFO",
    "WARNING",
    "ERROR",
    "CRITICAL",
)

# Substrings that indicate an unresolved placeholder serial-device value.
# Checked case-insensitively against the raw --serial-device argument.
_PLACEHOLDER_TOKENS: tuple[str, ...] = (
    "REPLACE",
    "PLACEHOLDER",
    "ACTUAL_H7_DEVICE",
)


# ---------------------------------------------------------------------------
# Configuration dataclass
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class BridgeConfig:
    """Immutable configuration resolved from defaults + CLI arguments."""

    host: str
    port: int
    serial_device: str
    baud_rate: int
    serial_read_timeout: float
    serial_write_timeout: float
    serial_reconnect_interval: float
    send_disconnect_event: bool
    disconnect_command: str
    tcp_nodelay: bool
    tcp_keepalive: bool
    log_level: str
    log_file: str | None
    # Parsed client-IP allowlist.  Empty tuple means allow all.
    allowed_client_networks: tuple[ipaddress.IPv4Network | ipaddress.IPv6Network, ...]


# ---------------------------------------------------------------------------
# Configuration validation
# ---------------------------------------------------------------------------

def validate_config(config: BridgeConfig) -> None:
    """Validate all configuration fields.

    Raises ``SystemExit(2)`` with a concise message on the first failure.
    Does not open serial ports, bind sockets, or start threads.
    """

    # --- host ---
    if not config.host:
        _fail("Host must not be empty")

    # --- port ---
    if not (1 <= config.port <= 65535):
        _fail(f"TCP port must be 1-65535, got {config.port}")

    # --- serial device ---
    if not config.serial_device:
        _fail("Serial device path must not be empty")
    _check_serial_placeholder(config.serial_device)

    # --- baud ---
    if config.baud_rate <= 0:
        _fail(f"Baud rate must be positive, got {config.baud_rate}")

    # --- timeouts ---
    if config.serial_read_timeout <= 0:
        _fail(f"Invalid read timeout: must be greater than zero, got {config.serial_read_timeout}")
    if config.serial_write_timeout <= 0:
        _fail(f"Invalid write timeout: must be greater than zero, got {config.serial_write_timeout}")
    if config.serial_reconnect_interval <= 0:
        _fail(f"Reconnect interval must be > 0, got {config.serial_reconnect_interval}")

    # --- allowlist networks (already parsed, just verify non-degenerate) ---
    # ip_network() already rejects garbage; nothing extra needed here.


def _check_serial_placeholder(value: str) -> None:
    """Reject obvious unresolved placeholder serial-device paths."""
    upper = value.upper()
    for token in _PLACEHOLDER_TOKENS:
        if token in upper:
            _fail(
                f"Invalid --serial-device value: unresolved placeholder detected "
                f"(contains {token!r})"
            )


def _fail(message: str) -> None:
    """Print a concise error and exit with code 2 (no traceback)."""
    print(f"error: {message}", file=sys.stderr)
    raise SystemExit(2)


class BridgeStartupError(RuntimeError):
    """Fatal listener/startup failure (exit code 1, no traceback)."""
    pass


# ---------------------------------------------------------------------------
# Bridge implementation
# ---------------------------------------------------------------------------

class TcpSerialBridge:
    """
    Single-client TCP server that bidirectionally forwards bytes between
    a TCP connection and a pyserial serial port.

    Lifecycle:
        bridge = TcpSerialBridge(config)
        bridge.run()          # blocks until shutdown signal
    """

    def __init__(self, config: BridgeConfig) -> None:
        self._cfg = config
        self._logger = logging.getLogger("bridge")

        # --- shutdown coordination ---
        self._shutdown_event = threading.Event()

        # --- serial state (protected by _serial_lock) ---
        self._serial_lock = threading.Lock()
        self._serial: Optional[serial.Serial] = None
        self._serial_connected = threading.Event()
        self._serial_write_lock = threading.Lock()

        # --- client state (protected by _client_lock) ---
        self._client_lock = threading.Lock()
        self._client_socket: Optional[socket.socket] = None
        self._client_addr: Optional[tuple[str, int]] = None
        self._disconnect_event_sent_for_session = False

        # --- listener socket ---
        self._listener: Optional[socket.socket] = None

        # --- worker threads (tracked for join) ---
        self._threads: list[threading.Thread] = []
        self._threads_lock = threading.Lock()

        # Pre-encode the disconnect event command once.
        self._disconnect_event_bytes: bytes = self._cfg.disconnect_command.encode("utf-8")

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> int:
        """Main blocking entry.  Returns exit code (0=clean, 1=fatal).

        Startup order:
            1. Create and bind TCP listener (fatal on failure)
            2. Start serial reconnect worker
            3. Enter accept loop

        This ordering ensures a bind failure never leaves a serial worker
        running and never produces a traceback for expected operational errors.
        """
        self._logger.info("Bridge starting")
        self._log_config()

        # --- Step 1: bind the listener before any background workers. ---
        # _create_listener() raises BridgeStartupError on failure.
        self._listener = self._create_listener()
        self._logger.info("TCP listener started on %s:%d",
                          self._cfg.host, self._cfg.port)

        # --- Step 2: now safe to start the serial reconnect worker. ---
        self._start_thread(self._serial_reconnect_loop, "serial-reconnect")

        # --- Step 3: enter the accept loop (blocks until shutdown). ---
        try:
            self._accept_loop()
        except BridgeStartupError:
            # Should not happen post-bind, but handle cleanly if it does.
            self._logger.critical("Unexpected startup error in accept loop")
            self.shutdown()
            return 1
        except Exception:
            self._logger.exception("Unexpected error in accept loop")
            self.shutdown()
            return 1

        self.shutdown()
        return 0

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _log_config(self) -> None:
        c = self._cfg
        if c.allowed_client_networks:
            allowlist_str = ",".join(str(n) for n in c.allowed_client_networks)
        else:
            allowlist_str = "ANY"
        self._logger.info(
            "Configuration: host=%s  port=%d  serial=%s  baud=%d  "
            "read_timeout=%.3f  write_timeout=%.3f  reconnect_interval=%.3f  "
            "disconnect_event_on_disconnect=%s  tcp_nodelay=%s  tcp_keepalive=%s  "
            "allowed_clients=%s  log_file=%s",
            c.host, c.port, c.serial_device, c.baud_rate,
            c.serial_read_timeout, c.serial_write_timeout,
            c.serial_reconnect_interval,
            c.send_disconnect_event, c.tcp_nodelay, c.tcp_keepalive,
            allowlist_str, c.log_file or "(stderr only)",
        )

    # ------------------------------------------------------------------
    # Thread helpers
    # ------------------------------------------------------------------

    def _start_thread(
        self, target: object, name: str, *, daemon: bool = True
    ) -> threading.Thread:
        t = threading.Thread(target=target, name=name, daemon=daemon)
        t.start()
        with self._threads_lock:
            self._threads.append(t)
        return t

    def _prune_threads(self) -> None:
        """Remove completed threads from the tracking list."""
        with self._threads_lock:
            self._threads = [t for t in self._threads if t.is_alive()]

    # ------------------------------------------------------------------
    # Serial management
    # ------------------------------------------------------------------

    def open_serial(self) -> bool:
        """
        Attempt to open (or re-open) the serial port.

        Returns True on success.  Caller must NOT hold _serial_lock.
        """
        self._logger.info("Opening serial device: %s", self._cfg.serial_device)
        try:
            ser = serial.Serial(
                port=self._cfg.serial_device,
                baudrate=self._cfg.baud_rate,
                timeout=self._cfg.serial_read_timeout,
                write_timeout=self._cfg.serial_write_timeout,
            )
        except Exception as exc:
            self._logger.error("Serial open failed: %s", exc)
            return False

        with self._serial_lock:
            # Close any previous instance cleanly.
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = ser

        self._serial_connected.set()
        self._logger.info("Serial connected: %s", self._cfg.serial_device)
        return True

    def close_serial(self) -> None:
        """Close the serial port if open.  Safe to call multiple times."""
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._serial_connected.clear()
            self._logger.info("Serial port closed")

    def _serial_reconnect_loop(self) -> None:
        """
        Background loop: keep trying to (re)open the serial port.

        Runs for the lifetime of the bridge.  When the port is open,
        this thread simply sleeps — actual I/O happens in the forwarding
        threads.  If an I/O thread detects a serial error it will call
        close_serial(), which clears _serial_connected and wakes this
        loop to retry.
        """
        while not self._shutdown_event.is_set():
            if self._get_serial() is None:
                if self.open_serial():
                    # Serial just came up.
                    pass
                else:
                    self._logger.warning(
                        "Serial unavailable — retrying in %.1f s",
                        self._cfg.serial_reconnect_interval,
                    )
                    self._shutdown_event.wait(
                        self._cfg.serial_reconnect_interval
                    )
                    continue

            # Serial is open — sleep briefly then re-check.
            self._shutdown_event.wait(0.5)

    def _get_serial(self) -> Optional[serial.Serial]:
        """Return current serial object or None (thread-safe snapshot)."""
        with self._serial_lock:
            return self._serial

    # ------------------------------------------------------------------
    # Centralized serial write
    # ------------------------------------------------------------------

    def _write_serial_all(self, data: bytes, context: str) -> bool:
        """Write complete byte sequence to serial under lock.

        Returns True if all bytes were written successfully.
        Returns False if serial is unavailable, write fails, or partial write.
        """
        with self._serial_write_lock:
            ser = self._get_serial()
            if ser is None:
                self._logger.debug(
                    "Serial write skipped (%s) — serial unavailable", context
                )
                return False
            try:
                remaining = data
                while remaining:
                    n = ser.write(remaining)
                    if n is None or n == 0:
                        self._logger.error(
                            "Serial write returned %r (%s) — closing serial",
                            n, context,
                        )
                        self.close_serial()
                        return False
                    if n < len(remaining):
                        self._logger.warning(
                            "Partial serial write %d/%d bytes (%s)",
                            n, len(remaining), context,
                        )
                        remaining = remaining[n:]
                    else:
                        remaining = b""
                return True
            except (serial.SerialException, serial.SerialTimeoutException,
                    OSError) as exc:
                self._logger.error(
                    "Serial write error: %s (%s) — closing serial", exc, context
                )
                self.close_serial()
                return False

    # ------------------------------------------------------------------
    # Client authorization
    # ------------------------------------------------------------------

    def _is_client_allowed(self, peer_ip_str: str) -> bool:
        """Check whether a peer IP is permitted by the allowlist.

        Returns True if the allowlist is empty (allow all) or if the peer
        belongs to at least one configured network.
        """
        if not self._cfg.allowed_client_networks:
            return True  # no allowlist configured — allow all
        try:
            peer_ip = ipaddress.ip_address(peer_ip_str)
        except ValueError:
            self._logger.warning(
                "Could not parse peer IP %r — rejecting", peer_ip_str
            )
            return False
        return any(peer_ip in net for net in self._cfg.allowed_client_networks)

    # ------------------------------------------------------------------
    # TCP listener
    # ------------------------------------------------------------------

    def _create_listener(self) -> socket.socket:
        """Create, bind, and listen on the TCP listener socket.

        Returns a valid listening socket.
        Raises ``BridgeStartupError`` on any failure (bind, listen, etc.).
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except OSError as exc:
            raise BridgeStartupError(
                f"TCP socket creation failed — {exc}"
            ) from exc

        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            sock.bind((self._cfg.host, self._cfg.port))
        except OSError as exc:
            sock.close()
            raise BridgeStartupError(
                f"TCP bind failed on {self._cfg.host}:{self._cfg.port} — {exc}"
            ) from exc

        try:
            sock.listen(1)
        except OSError as exc:
            sock.close()
            raise BridgeStartupError(
                f"TCP listen failed on {self._cfg.host}:{self._cfg.port} — {exc}"
            ) from exc

        return sock

    def _accept_loop(self) -> None:
        """Accept TCP clients continuously until shutdown.

        Additional clients while one is active are immediately rejected.
        Clients while serial is unavailable are immediately rejected.
        Clients not matching the allowlist are immediately rejected.
        """
        assert self._listener is not None
        self._listener.settimeout(1.0)  # so we can check shutdown_event

        while not self._shutdown_event.is_set():
            try:
                client_sock, client_addr = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._shutdown_event.is_set():
                    break
                raise

            # --- allowlist check (before any other gating) ---
            if not self._is_client_allowed(client_addr[0]):
                self._logger.warning(
                    "Unauthorized client rejected: %s:%d",
                    client_addr[0], client_addr[1],
                )
                try:
                    client_sock.close()
                except Exception:
                    pass
                continue

            # --- check serial availability ---
            if self._get_serial() is None:
                self._logger.warning(
                    "Rejected connection from %s:%d — serial unavailable",
                    client_addr[0], client_addr[1],
                )
                try:
                    client_sock.close()
                except Exception:
                    pass
                continue

            # --- single-client gating ---
            with self._client_lock:
                if self._client_socket is not None:
                    # Already have an active client — reject.
                    self._logger.warning(
                        "Rejected additional connection from %s:%d "
                        "(active client already connected)",
                        client_addr[0], client_addr[1],
                    )
                    try:
                        client_sock.close()
                    except Exception:
                        pass
                    continue

                # Accept as active controller.
                self._configure_client_socket(client_sock)
                self._client_socket = client_sock
                self._client_addr = client_addr
                self._disconnect_event_sent_for_session = False

                # Clear stale serial RX before starting forwarding.
                ser = self._get_serial()
                if ser is not None:
                    try:
                        ser.reset_input_buffer()
                    except Exception as exc:
                        self._logger.debug(
                            "reset_input_buffer failed: %s", exc
                        )

            self._logger.info(
                "Active client connected from %s:%d",
                client_addr[0], client_addr[1],
            )

            # Prune stale threads before spawning new ones.
            self._prune_threads()

            # Spawn forwarding threads for this client.
            self._start_thread(
                lambda s=client_sock, a=client_addr:
                    self._tcp_to_serial_loop(s, a),
                "tcp-to-serial",
            )
            self._start_thread(
                lambda s=client_sock, a=client_addr:
                    self._serial_to_tcp_loop(s, a),
                "serial-to-tcp",
            )

            # Small grace period then continue accepting.
            time.sleep(0.05)

    def _configure_client_socket(self, sock: socket.socket) -> None:
        """Apply socket options to the accepted client connection."""
        if self._cfg.tcp_nodelay:
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except (OSError, AttributeError) as exc:
                self._logger.debug("TCP_NODELAY not set: %s", exc)
        if self._cfg.tcp_keepalive:
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except (OSError, AttributeError) as exc:
                self._logger.debug("SO_KEEPALIVE not set: %s", exc)
        sock.settimeout(1.0)

    # ------------------------------------------------------------------
    # Forwarding loops
    # ------------------------------------------------------------------

    def _tcp_to_serial_loop(
        self, client_sock: socket.socket, client_addr: tuple[str, int]
    ) -> None:
        """Read bytes from TCP client and write them to serial."""
        while not self._shutdown_event.is_set():
            # Verify this is still the active client.
            with self._client_lock:
                if self._client_socket is not client_sock:
                    break  # stale reference

            try:
                data = client_sock.recv(_RECV_CHUNK_SIZE)
            except socket.timeout:
                continue
            except (OSError, ConnectionError):
                self._logger.debug("TCP recv error (client %s:%d)",
                                   client_addr[0], client_addr[1])
                break

            if not data:
                # Empty recv means the client closed the connection.
                self._logger.info(
                    "Client %s:%d disconnected (clean close)",
                    client_addr[0], client_addr[1],
                )
                break

            if not self._write_serial_all(data, "tcp-to-serial"):
                # Serial unavailable or write failed.
                self._logger.warning(
                    "Serial write failed — closing client %s:%d",
                    client_addr[0], client_addr[1],
                )
                break

            self._logger.debug("Forwarded %d bytes TCP -> Serial", len(data))

        # This forwarding loop is done — trigger disconnect handling.
        self._handle_client_disconnect(client_sock, client_addr)

    def _serial_to_tcp_loop(
        self, client_sock: socket.socket, client_addr: tuple[str, int]
    ) -> None:
        """Read bytes from serial and send them to the TCP client."""
        while not self._shutdown_event.is_set():
            # Wait for serial to be available.
            if not self._serial_connected.is_set():
                if self._shutdown_event.wait(0.2):
                    break
                continue

            # Verify this is still the active client.
            with self._client_lock:
                if self._client_socket is not client_sock:
                    break  # stale reference

            ser = self._get_serial()
            if ser is None:
                continue

            try:
                data = ser.read(_RECV_CHUNK_SIZE)
            except (serial.SerialException, OSError) as exc:
                self._logger.error("Serial read error: %s", exc)
                self.close_serial()
                # Serial went away — disconnect client for safety.
                self._logger.warning(
                    "Serial lost during active session — closing client "
                    "%s:%d for safety",
                    client_addr[0], client_addr[1],
                )
                break

            if not data:
                # read() returned empty due to timeout — normal, retry.
                continue

            # Check client is still the active one before writing.
            with self._client_lock:
                if self._client_socket is not client_sock:
                    break  # stale reference, a new client took over.

            try:
                client_sock.sendall(data)
            except (OSError, ConnectionError) as exc:
                self._logger.debug(
                    "TCP send error to %s:%d: %s",
                    client_addr[0], client_addr[1], exc,
                )
                break

            self._logger.debug("Forwarded %d bytes Serial -> TCP", len(data))

        self._handle_client_disconnect(client_sock, client_addr)

    # ------------------------------------------------------------------
    # Client disconnect handling
    # ------------------------------------------------------------------

    def _handle_client_disconnect(
        self, client_sock: socket.socket, client_addr: tuple[str, int]
    ) -> None:
        """
        Handle disconnection of the active TCP client.

        Thread-safe: multiple forwarding threads may call this
        concurrently for the same session — only the first caller
        performs the teardown.
        """
        with self._client_lock:
            # Only act if this socket is still the active client.
            if self._client_socket is not client_sock:
                return

            # Send disconnect event (once per session).
            self._send_disconnect_event("client disconnect")

            # Close the client socket.
            try:
                client_sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                client_sock.close()
            except OSError:
                pass

            # Clear client state.
            self._client_socket = None
            self._client_addr = None

            self._logger.info(
                "Active client %s:%d disconnected — controller "
                "ownership released",
                client_addr[0], client_addr[1],
            )

    def _send_disconnect_event(self, reason: str) -> None:
        """Send the disconnect event to H7 over serial.

        MUST be called with _client_lock held.
        Only sends once per client session.
        """
        if self._disconnect_event_sent_for_session:
            return
        self._disconnect_event_sent_for_session = True

        if not self._cfg.send_disconnect_event:
            self._logger.info(
                "Disconnect event disabled — not sending (%s)", reason
            )
            return

        if self._write_serial_all(self._disconnect_event_bytes,
                                  f"disconnect-event ({reason})"):
            self._logger.info("Disconnect event sent (%s)", reason)
        else:
            self._logger.warning(
                "Disconnect event could not be sent (%s)", reason
            )

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """
        Initiate graceful shutdown of the entire bridge.

        Safe to call multiple times (idempotent).
        """
        if self._shutdown_event.is_set():
            return
        self._logger.info("Graceful shutdown started")
        self._shutdown_event.set()

        # Attempt a final disconnect event if a client is still connected
        # and serial is available.
        with self._client_lock:
            if self._client_socket is not None:
                self._send_disconnect_event("bridge shutdown")

                try:
                    self._client_socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self._client_socket.close()
                except OSError:
                    pass
                self._client_socket = None
                self._client_addr = None

        # Close the listener socket.
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass
            self._listener = None

        # Close the serial port.
        self.close_serial()

        # Join worker threads with bounded timeout.
        with self._threads_lock:
            threads = list(self._threads)
        for t in threads:
            t.join(timeout=_JOIN_TIMEOUT)
            if t.is_alive():
                self._logger.warning(
                    "Thread '%s' did not exit within %.1f s",
                    t.name, _JOIN_TIMEOUT,
                )

        self._logger.info("Graceful shutdown completed")


# ---------------------------------------------------------------------------
# Argument parsing and validation
# ---------------------------------------------------------------------------

def _format_config_summary(config: BridgeConfig) -> str:
    """Return a human-readable summary of the effective configuration."""
    if config.allowed_client_networks:
        allowlist_str = ",".join(str(n) for n in config.allowed_client_networks)
    else:
        allowlist_str = "ANY"
    return (
        f"host={config.host}\n"
        f"port={config.port}\n"
        f"serial_device={config.serial_device}\n"
        f"baud={config.baud_rate}\n"
        f"read_timeout={config.serial_read_timeout}\n"
        f"write_timeout={config.serial_write_timeout}\n"
        f"reconnect_interval={config.serial_reconnect_interval}\n"
        f"disconnect_event_on_disconnect={config.send_disconnect_event}\n"
        f"tcp_nodelay={config.tcp_nodelay}\n"
        f"tcp_keepalive={config.tcp_keepalive}\n"
        f"log_file={config.log_file or '(stderr only)'}\n"
        f"allowed_clients={allowlist_str}"
    )


def parse_args(argv: list[str] | None = None) -> BridgeConfig:
    """Parse CLI arguments, validate, and return an immutable BridgeConfig."""

    parser = argparse.ArgumentParser(
        description=(
            "TCP-to-Serial bridge for STM32H723 rover. "
            "Runs on the Raspberry Pi and transparently forwards bytes "
            "between a single TCP client (the control PC) and the H7 "
            "serial port."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  # Remote GUI: run this on the Pi; GUI connects to the Pi LAN IP.\n"
            "  python3 tcp_uart_bridge.py \\\n"
            "    --host 0.0.0.0 --port 5000 \\\n"
            "    --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_XXXX-if02 \\\n"
            "    --baud 115200 --allow-client 192.168.50.10/32 \\\n"
            "    --log-file /tmp/bridge.log\n"
            "\n"
            "  # 127.0.0.1 is only valid when the GUI also runs on this Pi.\n"
            "  python3 tcp_uart_bridge.py \\\n"
            "    --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_XXXX-if02 \\\n"
            "    --check-config\n"
            "\n"
            "Run serial checks on the Raspberry Pi. Find the persistent path:\n"
            "  ls -l /dev/serial/by-id/"
        ),
    )

    parser.add_argument(
        "--host",
        type=str,
        default=DEFAULT_LISTEN_HOST,
        help=f"TCP listen address (default: {DEFAULT_LISTEN_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_LISTEN_PORT,
        help=f"TCP listen port, 1-65535 (default: {DEFAULT_LISTEN_PORT})",
    )
    parser.add_argument(
        "--serial-device",
        type=str,
        required=True,
        help="Serial device path (REQUIRED) — prefer /dev/serial/by-id/...",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"Serial baud rate (default: {DEFAULT_BAUD_RATE})",
    )
    parser.add_argument(
        "--serial-read-timeout",
        type=float,
        default=DEFAULT_SERIAL_READ_TIMEOUT,
        help=(
            "Serial read timeout in seconds "
            f"(default: {DEFAULT_SERIAL_READ_TIMEOUT})"
        ),
    )
    parser.add_argument(
        "--serial-write-timeout",
        type=float,
        default=DEFAULT_SERIAL_WRITE_TIMEOUT,
        help=(
            "Serial write timeout in seconds "
            f"(default: {DEFAULT_SERIAL_WRITE_TIMEOUT})"
        ),
    )
    parser.add_argument(
        "--serial-reconnect-interval",
        type=float,
        default=DEFAULT_SERIAL_RECONNECT_INTERVAL,
        help=(
            "Seconds between serial reconnect attempts "
            f"(default: {DEFAULT_SERIAL_RECONNECT_INTERVAL})"
        ),
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default=DEFAULT_LOG_LEVEL,
        choices=list(_VALID_LOG_LEVELS),
        help=f"Logging level (default: {DEFAULT_LOG_LEVEL})",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help=(
            "Also append logs to this file while preserving stderr output "
            "(example: /tmp/bridge.log)."
        ),
    )
    parser.add_argument(
        "--disable-disconnect-event",
        "--disable-stop-on-disconnect",
        action="store_true",
        default=not DEFAULT_SEND_DISCONNECT_EVENT,
        help="Disable sending the disconnect event when the TCP client disconnects",
    )
    parser.add_argument(
        "--allow-client",
        action="append",
        dest="allow_clients",
        metavar="NETWORK",
        default=None,
        help=(
            "Allow only this IP/CIDR to connect (repeatable). "
            "Plain addresses are treated as /32 (IPv4) or /128 (IPv6). "
            "Omit to allow all source IPs."
        ),
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        default=False,
        help="Validate configuration and print effective values, then exit (no I/O).",
    )

    args = parser.parse_args(argv)

    # --- normalize and validate ---

    serial_device = args.serial_device.strip()
    log_file = args.log_file.strip() if args.log_file else None
    if args.log_file is not None and not log_file:
        parser.error("Empty --log-file value")

    # Parse allowlist early so validate_config sees the final tuple.
    parsed_networks: list[ipaddress.IPv4Network | ipaddress.IPv6Network] = []
    if args.allow_clients:
        for raw in args.allow_clients:
            raw = raw.strip()
            if not raw:
                parser.error("Empty --allow-client value")
            try:
                net = ipaddress.ip_network(raw, strict=False)
                parsed_networks.append(net)
            except ValueError:
                parser.error(
                    f"Invalid --allow-client value: {raw!r} "
                    f"(expected IPv4/IPv6 address or CIDR network)"
                )

    config = BridgeConfig(
        host=args.host,
        port=args.port,
        serial_device=serial_device,
        baud_rate=args.baud,
        serial_read_timeout=args.serial_read_timeout,
        serial_write_timeout=args.serial_write_timeout,
        serial_reconnect_interval=args.serial_reconnect_interval,
        send_disconnect_event=not args.disable_disconnect_event,
        disconnect_command=DEFAULT_DISCONNECT_COMMAND,
        tcp_nodelay=DEFAULT_TCP_NODELAY,
        tcp_keepalive=DEFAULT_TCP_KEEPALIVE,
        log_level=args.log_level,
        log_file=log_file,
        allowed_client_networks=tuple(parsed_networks),
    )

    # Run centralized validation (may SystemExit(2)).
    validate_config(config)

    # --check-config: print and exit without starting workers.
    if args.check_config:
        print("Configuration valid")
        print(_format_config_summary(config))
        raise SystemExit(0)

    return config


# ---------------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------------

_bridge_ref: Optional[TcpSerialBridge] = None


def _signal_handler(signum: int, _frame: object) -> None:
    """Handle SIGINT / SIGTERM by requesting bridge shutdown."""
    sig_name = signal.Signals(signum).name
    logging.getLogger("bridge").info("Received %s — shutting down", sig_name)
    if _bridge_ref is not None:
        _bridge_ref.shutdown()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    """Entry point.  Returns 0 on clean exit, non-zero on fatal error."""

    config = parse_args(argv)

    # Always preserve stderr output.  No file is created unless --log-file
    # was explicitly supplied.
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stderr)]
    if config.log_file:
        try:
            handlers.append(logging.FileHandler(config.log_file))
        except OSError as exc:
            _fail(f"Could not open --log-file {config.log_file!r}: {exc}")

    logging.basicConfig(
        level=getattr(logging, config.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=handlers,
    )

    # Register signal handlers.
    global _bridge_ref
    bridge = TcpSerialBridge(config)
    _bridge_ref = bridge

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _signal_handler)

    try:
        exit_code = bridge.run()
    except BridgeStartupError as exc:
        # Fatal listener failure — concise log, no traceback, exit 1.
        logging.getLogger("bridge").critical("%s", exc)
        bridge.shutdown()
        exit_code = 1
    except KeyboardInterrupt:
        bridge.shutdown()
        exit_code = 0

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
