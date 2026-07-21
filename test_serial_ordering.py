#!/usr/bin/env python3
"""
test_serial_ordering.py — Verify bridge serial TX ordering guarantee.

When a TCP client sends a command and then disconnects, the bridge must
write the command bytes to serial BEFORE the disconnect event.  This test
uses a mock serial port and a mock TCP socket to verify deterministic
ordering without real hardware.

Expected ordering:
    1. mode auto\r\n   (from TCP client)
    2. pc_disconnect\r\n  (injected by bridge on disconnect)
"""

from __future__ import annotations

import socket
import threading
import time
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Mock serial port that records all writes with timestamps
# ---------------------------------------------------------------------------

class MockSerial:
    """A mock serial port that records every write() call in order."""

    def __init__(self):
        self.writes: list[bytes] = []
        self.write_timestamps: list[float] = []
        self._lock = threading.Lock()
        self.is_open = True

    def write(self, data: bytes) -> int:
        with self._lock:
            self.writes.append(bytes(data))
            self.write_timestamps.append(time.monotonic())
        return len(data)

    def read(self, size: int) -> bytes:
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False

    def get_writes(self) -> list[bytes]:
        with self._lock:
            return list(self.writes)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_serial_ordering():
    """Verify mode command precedes disconnect event on serial."""

    # Import the bridge (from the repo root)
    import sys
    sys.path.insert(0, "/home/garth/H7-DMA")
    from tcp_uart_bridge import TcpSerialBridge, BridgeConfig

    # Create config
    config = BridgeConfig(
        host="127.0.0.1",
        port=0,  # not used — we won't bind
        serial_device="/dev/null",
        baud_rate=115200,
        serial_read_timeout=0.05,
        serial_write_timeout=0.2,
        serial_reconnect_interval=1.0,
        send_disconnect_event=True,
        disconnect_command="pc_disconnect\r\n",
        tcp_nodelay=True,
        tcp_keepalive=True,
        log_level="WARNING",
        log_file=None,
        allowed_client_networks=(),
    )

    bridge = TcpSerialBridge(config)

    # Inject mock serial
    mock_ser = MockSerial()
    bridge._serial = mock_ser
    bridge._serial_connected.set()

    # Create a connected TCP socket pair
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", 0))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]

    client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_sock.connect(("127.0.0.1", port))
    accepted_sock, _ = server_sock.accept()

    # Simulate an active client session
    with bridge._client_lock:
        bridge._client_socket = accepted_sock
        bridge._client_addr = ("127.0.0.1", port)
        bridge._disconnect_event_sent_for_session = False

    # Simulate: TCP client sends "mode auto\r\n"
    # The bridge's _tcp_to_serial_loop would call _write_serial_all.
    # We simulate this directly:
    cmd_bytes = b"mode auto\r\n"
    bridge._write_serial_all(cmd_bytes, "tcp-to-serial")

    # Now simulate: client disconnects → bridge sends disconnect event
    bridge._send_disconnect_event("client disconnect")

    # Verify ordering
    writes = mock_ser.get_writes()

    print(f"Total serial writes: {len(writes)}")
    for i, w in enumerate(writes):
        print(f"  [{i}] {w!r}")

    assert len(writes) == 2, f"Expected 2 writes, got {len(writes)}"
    assert writes[0] == b"mode auto\r\n", \
        f"First write should be 'mode auto\\r\\n', got {writes[0]!r}"
    assert writes[1] == b"pc_disconnect\r\n", \
        f"Second write should be 'pc_disconnect\\r\\n', got {writes[1]!r}"

    # Verify timestamps are ordered
    assert mock_ser.write_timestamps[0] <= mock_ser.write_timestamps[1], \
        "Timestamps out of order"

    print("\nPASS: Serial TX ordering is correct.")
    print("  1. mode auto\\r\\n  (first)")
    print("  2. pc_disconnect\\r\\n  (second)")

    # Cleanup
    client_sock.close()
    accepted_sock.close()
    server_sock.close()


if __name__ == "__main__":
    test_serial_ordering()
