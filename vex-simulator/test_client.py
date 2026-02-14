"""Quick test client — sends raw voltage commands to the simulator."""

import json
import socket
import threading
import time


def send(sock: socket.socket, msg: dict) -> None:
    sock.sendall((json.dumps(msg) + "\n").encode())


def _drain(sock: socket.socket, stop: threading.Event) -> None:
    """Continuously drain incoming state updates so the TCP buffer doesn't fill."""
    while not stop.is_set():
        try:
            data = sock.recv(8192)
            if not data:
                break
        except (socket.timeout, OSError):
            pass


def main() -> None:
    sock = socket.create_connection(("127.0.0.1", 9090))
    sock.settimeout(0.2)

    # Read handshake
    try:
        data = sock.recv(4096)
        print("Handshake:", data.decode().strip())
    except socket.timeout:
        print("No handshake received")

    # Start a background thread to drain state updates
    stop_drain = threading.Event()
    drain_thread = threading.Thread(target=_drain, args=(sock, stop_drain), daemon=True)
    drain_thread.start()

    # ── Drive forward at full voltage for 3 seconds ──
    print("Driving forward (voltage=127)...")
    for p in range(1, 7):
        send(sock, {"type": "motor_move", "port": p, "voltage": 127})
    time.sleep(3)

    # ── Turn right for 1.5 seconds ──
    print("Turning right (L=80, R=-80)...")
    for p in (1, 2, 3):
        send(sock, {"type": "motor_move", "port": p, "voltage": 80})
    for p in (4, 5, 6):
        send(sock, {"type": "motor_move", "port": p, "voltage": -80})
    time.sleep(1.5)

    # ── Drive forward at half voltage for 2 seconds ──
    print("Driving forward (voltage=64)...")
    for p in range(1, 7):
        send(sock, {"type": "motor_move", "port": p, "voltage": 64})
    time.sleep(2)

    # ── Coast to stop ──
    print("Stopping (voltage=0, coast)...")
    for p in range(1, 7):
        send(sock, {"type": "motor_move", "port": p, "voltage": 0})
    time.sleep(2)

    stop_drain.set()
    drain_thread.join(timeout=1)
    sock.close()
    print("Done.")


if __name__ == "__main__":
    main()
