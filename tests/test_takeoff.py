"""Readiness, confirmed arming, hold, and loss-of-feedback regression checks."""

from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers/drone_telemetry'))
from auto_takeoff import AutoTakeoff
from betaflight_bridge import BetaflightBridge, KeyboardPilot, RC, decode_status

IDENTITY = [1, 0, 0, 0, 1, 0, 0, 0, 1]


class TakeoffTests(unittest.TestCase):
    def update(self, pilot, now, status, position=(0, 0, 0.055), keys=set(), connected=True):
        return pilot.update(keys, now, connected, status, position, (0, 0, 0), IDENTITY, (0, 0, 0), 0.02)

    def test_grace_period_and_confirmed_arm_before_throttle(self):
        pilot = AutoTakeoff()
        self.update(pilot, 0, (False, 512, 0))
        self.update(pilot, 3, (False, 512, 3))
        self.assertEqual(pilot.channels[4], 1000)
        self.update(pilot, 5, (False, 0, 5))
        self.assertEqual(pilot.phase, 'arming')
        self.update(pilot, 5.2, (False, 0, 5.2))
        self.assertEqual(pilot.channels[2], 1000)
        self.update(pilot, 5.3, (True, 0, 5.3))
        self.update(pilot, 6, (True, 0, 6))
        self.assertGreater(pilot.channels[2], 1500)
        self.assertLess(pilot.channels[2], 1800)

    def test_hover_and_commands_preserve_target_when_released(self):
        pilot = AutoTakeoff()
        self.update(pilot, 0, (False, 0, 0))
        self.update(pilot, 2, (False, 0, 2))
        self.update(pilot, 2.1, (True, 0, 2.1))
        self.update(pilot, 9, (True, 0, 9), (0, 0, 3))
        self.update(pilot, 10.1, (True, 0, 10.1), (0, 0, 3))
        self.assertEqual(pilot.phase, 'holding')
        self.update(pilot, 10.2, (True, 0, 10.2), (0, 0, 3), {'RIGHT'})
        target = list(pilot.target)
        self.update(pilot, 10.3, (True, 0, 10.3), (0, 0, 3))
        self.assertEqual(pilot.target, target)
        self.assertGreater(target[0], 0)
        self.update(pilot, 10.4, None, (0, 0, 3))
        self.assertFalse(pilot.armed)
        self.assertEqual(pilot.channels[4], 1000)
        self.update(pilot, 13, (False, 0, 13), (0, 0, 3))
        self.assertFalse(pilot.armed)

    def test_manual_arm_waits_for_firmware_ready(self):
        pilot = KeyboardPilot()
        pilot.update(set(), 0, True, ready=False)
        pilot.update({'R'}, 3, True, ready=False)
        self.assertFalse(pilot.armed)

    def test_excessive_tilt_aborts_and_space_cannot_rearm(self):
        pilot = AutoTakeoff()
        pilot.phase = 'holding'
        pilot.armed = True
        pilot.takeoff_time = 0
        pilot.target = [0, 0, 3]
        tilted = [1, 0, 0, 0, 0, -1, 0, 1, 0]
        pilot.update(set(), 10, True, (True, 0, 10), (0, 0, 3),
                     (0, 0, 0), tilted, (0, 0, 0), 0.02)
        self.assertEqual(pilot.fault_reason, 'excessive_tilt')
        self.assertEqual(pilot.channels[2:5:2], [1000, 1000])
        self.update(pilot, 11, (False, 0, 11), keys={'SPACE', 'R'})
        self.assertFalse(pilot.armed)

    def test_loop_stall_does_not_refresh_queued_motor_packets(self):
        with patch('betaflight_bridge.socket.socket') as factory:
            sock = factory.return_value
            bridge = BetaflightBridge()
            packet = struct.pack('<4f', 0.5, 0.5, 0.5, 0.5)
            sock.recvfrom.side_effect = [(packet, ('127.0.0.1', 9002)), BlockingIOError()]
            bridge.read_motors(1)
            sock.recvfrom.side_effect = [(packet, ('127.0.0.1', 9002)), BlockingIOError()]
            outputs, age = bridge.read_motors(2)
            self.assertEqual(outputs, (0,) * 4)
            self.assertIsNone(age)

    def test_status_flags(self):
        payload = bytearray(24)
        struct.pack_into('<I', payload, 6, 1)
        struct.pack_into('<I', payload, 17, 1 << 29)
        self.assertEqual(decode_status(payload), (True, 1 << 29))
        with self.assertRaises(ValueError):
            decode_status(payload[:10])

    def test_rc_heartbeat_disarms_stale_control(self):
        with patch('betaflight_bridge.socket.socket'), patch('betaflight_bridge.time.monotonic', return_value=10):
            bridge = BetaflightBridge()
            channels = [1500, 1500, 1550, 1500, 2000, 2000] + [1000] * 10
            bridge.send_rc(2, channels)
            self.assertEqual(RC.unpack(bridge.rc_packet(10.1))[1:], tuple(channels))
            stale = RC.unpack(bridge.rc_packet(10.3))[1:]
            self.assertEqual((stale[2], stale[4]), (1000, 1000))
            bridge.close()


if __name__ == '__main__':
    unittest.main()
