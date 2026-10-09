"""Strict sensor-only Betaflight UDP protocol and keyboard pilot state.

Adapted from the learning repository's Module 08 protocol/client implementation.
"""

import math
import socket
import struct
import threading
import time

FDM = struct.Struct("<18d")
RC = struct.Struct("<d16H")
MOTORS = struct.Struct("<4f")
MOTOR_NAMES = ("m4_motor", "m1_motor", "m2_motor", "m3_motor")
TIMEOUT_S = 0.25


def pack_sensors(timestamp, gyro, acceleration, pressure):
    x, y, z = gyro
    ax, ay, az = acceleration
    values = (timestamp, x, -y, -z, -ax, -ay, -az,
              1.0, 0.0, 0.0, 0.0, *([0.0] * 6), pressure)
    if not all(math.isfinite(value) for value in values) or timestamp < 0 or pressure <= 0:
        raise ValueError("Invalid sensor sample")
    return FDM.pack(*values)


def pack_rc(timestamp, channels):
    if not math.isfinite(timestamp) or timestamp < 0:
        raise ValueError("Invalid RC timestamp")
    if len(channels) != 16 or any(type(value) is not int or not 1000 <= value <= 2000 for value in channels):
        raise ValueError("RC requires sixteen integer channels in [1000, 2000]")
    return RC.pack(timestamp, *channels)


def unpack_motors(packet):
    if len(packet) != MOTORS.size:
        raise ValueError("Motor packet must contain four floats")
    values = MOTORS.unpack(packet)
    if not all(math.isfinite(value) and 0 <= value <= 1 for value in values):
        raise ValueError("Motor output must be finite and in [0, 1]")
    return values


class BetaflightBridge:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("127.0.0.1", 9002))
            self.socket.setblocking(False)
        except OSError:
            self.socket.close()
            raise
        self.received_at = None
        self.motors = (0.0,) * 4
        self.last_poll = None
        self.rc_frame = None
        self.rc_stop = threading.Event()
        self.rc_thread = None

    def send_sensors(self, timestamp, gyro, acceleration, pressure):
        self.socket.sendto(pack_sensors(timestamp, gyro, acceleration, pressure), ("127.0.0.1", 9003))

    def send_rc(self, timestamp, channels):
        packet = pack_rc(timestamp, channels)
        self.rc_frame = (packet, time.monotonic())
        if self.rc_thread is None or channels[4] == 1000:
            self.socket.sendto(packet, ("127.0.0.1", 9004))

    def start_rc(self):
        # GUI/physics scheduling must not interrupt the receiver heartbeat.
        def run():
            while not self.rc_stop.is_set():
                self.socket.sendto(self.rc_packet(time.monotonic()), ('127.0.0.1', 9004))
                self.rc_stop.wait(0.02)
        self.rc_thread = threading.Thread(target=run, daemon=True)
        self.rc_thread.start()

    def rc_packet(self, now):
        frame = self.rc_frame
        if frame and now - frame[1] <= TIMEOUT_S:
            return frame[0]
        return pack_rc(0, [1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 10)

    def read_motors(self, now):
        stalled = self.last_poll is not None and now - self.last_poll > TIMEOUT_S
        self.last_poll = now
        if stalled:
            self.received_at = None
            self.motors = (0.0,) * 4
        for _ in range(64):
            try:
                packet, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if address[0] != "127.0.0.1":
                continue
            if stalled:
                continue
            try:
                motors = unpack_motors(packet)
            except ValueError:
                continue
            self.motors = motors
            self.received_at = now
        age = None if self.received_at is None else now - self.received_at
        return (self.motors if age is not None and age <= TIMEOUT_S else (0.0,) * 4), age

    def close(self):
        self.rc_stop.set()
        if self.rc_thread is not None:
            self.rc_thread.join(timeout=1)
        self.socket.close()


class KeyboardPilot:
    def __init__(self):
        self.armed = False
        self.throttle = 0.0
        self.stable_since = None
        self.previous_keys = set()
        self.channels = [1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 10

    def disarm(self):
        self.armed = False
        self.throttle = 0.0
        self.channels[2] = self.channels[4] = 1000

    def update(self, keys, now, connected, ready=True):
        if not connected:
            self.stable_since = None
            self.disarm()
        elif self.stable_since is None:
            self.stable_since = now
        if "SPACE" in keys:
            self.disarm()
        elif ("R" in keys and "R" not in self.previous_keys and connected
              and now - self.stable_since >= 2 and self.throttle == 0 and ready):
            self.armed = True
        if self.armed:
            self.throttle = max(0.0, min(1.0, self.throttle + 0.01 * (("W" in keys) - ("S" in keys))))
        self.channels = [
            1500 + 100 * (("RIGHT" in keys) - ("LEFT" in keys)),
            1500 + 100 * (("UP" in keys) - ("DOWN" in keys)),
            round(1000 + 1000 * self.throttle),
            1500 + 100 * (("D" in keys) - ("A" in keys)),
            2000 if self.armed else 1000, 2000,
        ] + [1000] * 10
        self.previous_keys = set(keys)
        return self.channels


def decode_status(payload):
    if len(payload) < 21:
        raise ValueError("Truncated MSP_STATUS")
    extra = payload[15] & 15
    if len(payload) < 21 + extra:
        raise ValueError("Truncated arming flags")
    return bool(struct.unpack_from('<I', payload, 6)[0] & 1), struct.unpack_from('<I', payload, 17 + extra)[0]


class StatusMonitor:
    """Poll read-only MSP_STATUS away from the physics/RC loop."""

    def __init__(self):
        self.latest = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        def receive(sock, count):
            result = b''
            while len(result) < count:
                data = sock.recv(count - len(result))
                if not data:
                    raise ConnectionError("MSP connection closed")
                result += data
            return result

        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', 5761), timeout=0.2) as sock:
                    while not self.stop.is_set():
                        sock.sendall(b'$M<' + bytes((0, 101, 101)))
                        header = receive(sock, 5)
                        if header[:3] != b'$M>' or header[4] != 101:
                            raise ValueError("Unexpected MSP response")
                        data = receive(sock, header[3] + 1)
                        checksum = header[3] ^ header[4]
                        for value in data[:-1]:
                            checksum ^= value
                        if checksum != data[-1]:
                            raise ValueError("Invalid MSP checksum")
                        armed, flags = decode_status(data[:-1])
                        self.latest = (armed, flags, time.monotonic())
                        self.stop.wait(0.1)
            except (OSError, ValueError):
                self.stop.wait(0.2)

    def state(self, now):
        latest = self.latest
        return latest if latest is not None and now - latest[2] < 0.5 else None

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
