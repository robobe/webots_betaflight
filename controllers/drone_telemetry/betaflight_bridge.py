"""Strict sensor-only Betaflight UDP protocol and keyboard pilot state.

Adapted from the learning repository's Module 08 protocol/client implementation.
"""

import math
import socket
import struct

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

    def send_sensors(self, timestamp, gyro, acceleration, pressure):
        self.socket.sendto(pack_sensors(timestamp, gyro, acceleration, pressure), ("127.0.0.1", 9003))

    def send_rc(self, timestamp, channels):
        self.socket.sendto(pack_rc(timestamp, channels), ("127.0.0.1", 9004))

    def read_motors(self, now):
        for _ in range(64):
            try:
                packet, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if address[0] != "127.0.0.1":
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

    def update(self, keys, now, connected):
        if not connected:
            self.stable_since = None
            self.disarm()
        elif self.stable_since is None:
            self.stable_since = now
        if "SPACE" in keys:
            self.disarm()
        elif ("R" in keys and "R" not in self.previous_keys and connected
              and now - self.stable_since >= 2 and self.throttle == 0):
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
