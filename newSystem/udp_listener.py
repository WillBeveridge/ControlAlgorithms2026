"""
udp_listener.py - UDP Broadcast Listener (Debug Tool)
Run this on a second machine (or second terminal) on the same network
to verify the UDP server is broadcasting correctly.

Usage:
    python udp_listener.py

Make sure this machine is on the same subnet as the server.
"""

import socket
import struct
import math
import time

# ── Must match udp.py ────────────────────────────────────────────────────────
LISTEN_PORT    = 5005
LISTEN_IP      = ""        # "" = listen on all interfaces (catches broadcasts)

HEADER_FORMAT    = "!cIB"   # command (1 byte), packet_number (4 bytes), num_robots (1 byte)
ROBOT_FORMAT     = "!Bfff"  # robot_id (1 byte), x (float), y (float), theta (float)
HEADER_SIZE      = struct.calcsize(HEADER_FORMAT)
ROBOT_ENTRY_SIZE = struct.calcsize(ROBOT_FORMAT)
# ─────────────────────────────────────────────────────────────────────────────


def decode_packet(data: bytes):
    """Parse raw UDP packet into (command, packet_number, robots_dict)."""
    if len(data) < HEADER_SIZE:
        return None, None, None

    command_byte, packet_number, num_robots = struct.unpack_from(HEADER_FORMAT, data, 0)
    command = "RUN" if command_byte == b'R' else "STOP"

    robots = {}
    offset = HEADER_SIZE
    for _ in range(num_robots):
        if offset + ROBOT_ENTRY_SIZE > len(data):
            print("[WARN] Packet truncated — fewer robot entries than num_robots claims.")
            break
        robot_id, x, y, theta = struct.unpack_from(ROBOT_FORMAT, data, offset)
        robots[robot_id] = (x, y, theta)
        offset += ROBOT_ENTRY_SIZE

    # Check for leftover bytes (indicates a format mismatch)
    leftover = len(data) - offset
    if leftover != 0:
        print(f"[WARN] {leftover} unexpected leftover bytes — possible format mismatch.")

    return command, packet_number, robots


def print_packet(command, packet_number, robots, recv_time, sender_addr):
    """Pretty-print a decoded packet."""
    print(f"\n{'─' * 55}")
    print(f"  From         : {sender_addr[0]}:{sender_addr[1]}")
    print(f"  Received at  : {recv_time:.3f}s")
    print(f"  Packet #     : {packet_number}")
    print(f"  Command      : {command}")
    print(f"  Num robots   : {len(robots)}")
    for robot_id, (x, y, theta) in robots.items():
        print(f"    Robot {robot_id:>2}  →  x={x:+.3f}m  y={y:+.3f}m  θ={math.degrees(theta):+.1f}°")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))

    print(f"[Listener] Listening for UDP broadcasts on port {LISTEN_PORT}...")
    print(f"[Listener] Waiting for packets — Ctrl+C to stop.\n")

    start_time   = time.time()
    packet_count = 0
    last_packet_number = None
    dropped_count = 0

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            recv_time = time.time() - start_time
            packet_count += 1

            command, packet_number, robots = decode_packet(data)

            if command is None:
                print(f"[ERROR] Received {len(data)} bytes but could not decode header.")
                continue

            # Detect dropped or out-of-order packets
            if last_packet_number is not None:
                expected = (last_packet_number + 1) % (2**32)
                if packet_number != expected:
                    gap = (packet_number - last_packet_number) % (2**32)
                    dropped_count += gap - 1
                    print(f"[WARN] Packet gap detected! Expected #{expected}, got #{packet_number}. "
                          f"~{gap - 1} packet(s) missed. Total dropped: {dropped_count}")
            last_packet_number = packet_number

            print_packet(command, packet_number, robots, recv_time, addr)
            print(f"  Total received : {packet_count}  |  Total dropped : {dropped_count}")

    except KeyboardInterrupt:
        print(f"\n[Listener] Stopped.")
        print(f"  Total packets received : {packet_count}")
        print(f"  Total packets dropped  : {dropped_count}")
        elapsed = time.time() - start_time
        if elapsed > 0:
            print(f"  Average rate           : {packet_count / elapsed:.1f} Hz")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
