"""Protocol, watchdog, and pilot checks without Webots or a running firmware."""

import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers/drone_telemetry"))
from betaflight_bridge import BetaflightBridge, FDM, RC, MOTOR_NAMES, KeyboardPilot, pack_rc, pack_sensors, unpack_motors


class ProtocolTests(unittest.TestCase):
    def test_sensor_axis_signs_and_no_truth(self):
        packet = pack_sensors(1.25, (1, 2, 3), (4, 5, 6), 101325)
        self.assertEqual(len(packet), 144)
        self.assertEqual(FDM.unpack(packet), (1.25, 1, -2, -3, -4, -5, -6,
                                            1, 0, 0, 0, *([0] * 6), 101325))
        with self.assertRaises(ValueError):
            pack_sensors(0, (math.nan, 0, 0), (0, 0, 9.81), 101325)

    def test_rc_layout_and_limits(self):
        channels = [1500, 1500, 1000, 1500, 2000, 2000] + [1000] * 10
        self.assertEqual(len(pack_rc(1, channels)), 40)
        self.assertEqual(RC.unpack(pack_rc(1, channels))[1:], tuple(channels))
        for bad in (channels[:-1], [999] * 16, [1500.0] * 16):
            with self.assertRaises(ValueError):
                pack_rc(1, bad)

    def test_motor_validation_and_iris_mapping(self):
        outputs = unpack_motors(struct.pack('<4f', 0, 0.25, 0.5, 1))
        self.assertEqual(dict(zip(MOTOR_NAMES, outputs)),
                         {'m4_motor': 0, 'm1_motor': 0.25, 'm2_motor': 0.5, 'm3_motor': 1})
        for packet in (b'', struct.pack('<4f', math.nan, 0, 0, 0),
                       struct.pack('<4f', -0.1, 0, 0, 0), struct.pack('<4f', 1.1, 0, 0, 0)):
            with self.assertRaises(ValueError):
                unpack_motors(packet)

    def test_iris_mixer_feedback_signs(self):
        # Iris geometry in QUADX order; Webots applies reaction torque opposite shaft speed.
        positions = ((-0.13, -0.20), (0.13, -0.22), (-0.13, 0.20), (0.13, 0.22))
        multipliers = (-1, 1, 1, -1)
        mixer = ((-1, -1, 1, 1), (1, -1, 1, -1), (-1, 1, 1, -1))
        for axis, coefficients in enumerate(mixer):
            thrust = [0.0012 * (50 + 5 * coefficient) ** 2 for coefficient in coefficients]
            torque = (
                sum(position[1] * force for position, force in zip(positions, thrust)),
                -sum(position[0] * force for position, force in zip(positions, thrust)),
                -sum(0.0005 * multiplier * (50 + 5 * coefficient) ** 2
                     for multiplier, coefficient in zip(multipliers, coefficients)),
            )
            # Positive PID output produces a positive measured rate on each BF axis.
            measured = (torque[0], torque[1], -torque[2])
            self.assertGreater(measured[axis], 0)

    def test_watchdog_and_invalid_packets(self):
        with patch('betaflight_bridge.socket.socket') as factory:
            sock = factory.return_value
            bridge = BetaflightBridge()
            packet = struct.pack('<4f', 0.25, 0.25, 0.25, 0.25)
            sock.recvfrom.side_effect = [(packet, ('127.0.0.1', 9002)), BlockingIOError()]
            self.assertEqual(bridge.read_motors(10), ((0.25,) * 4, 0))
            sock.recvfrom.side_effect = [(b'bad', ('127.0.0.1', 9002)), BlockingIOError()]
            self.assertEqual(bridge.read_motors(10.2)[0], (0.25,) * 4)
            sock.recvfrom.side_effect = BlockingIOError()
            self.assertEqual(bridge.read_motors(10.3)[0], (0,) * 4)
            self.assertEqual(bridge.received_at, 10)
            sock.recvfrom.side_effect = [(packet, ('127.0.0.1', 9002))] * 65
            before = sock.recvfrom.call_count
            bridge.read_motors(11)
            self.assertEqual(sock.recvfrom.call_count - before, 64)
            bridge.close()
            sock.close.assert_called_once()


class PilotTests(unittest.TestCase):
    def test_arm_sequence_sticks_disarm_and_reconnect(self):
        pilot = KeyboardPilot()
        pilot.update({'R'}, 0, True)
        self.assertFalse(pilot.armed)
        pilot.update(set(), 2, True)
        pilot.update({'R'}, 2.02, True)
        self.assertTrue(pilot.armed)
        channels = pilot.update({'W', 'RIGHT', 'UP', 'D'}, 2.04, True)
        self.assertEqual(channels[:6], [1600, 1600, 1010, 1600, 2000, 2000])
        self.assertEqual(pilot.update(set(), 2.06, True)[:4], [1500, 1500, 1010, 1500])
        pilot.update(set(), 3, False)
        self.assertFalse(pilot.armed)
        self.assertEqual(pilot.channels[2], 1000)
        pilot.update({'R'}, 4, True)
        self.assertFalse(pilot.armed)
        pilot.update(set(), 6, True)
        pilot.update({'R'}, 6.02, True)
        self.assertTrue(pilot.armed)
        pilot.update({'SPACE', 'W', 'R'}, 6.04, True)
        self.assertFalse(pilot.armed)
        self.assertEqual(pilot.throttle, 0)

    def test_throttle_is_bounded(self):
        pilot = KeyboardPilot()
        pilot.update(set(), 0, True)
        pilot.update({'R'}, 2, True)
        for index in range(150):
            pilot.update({'W'}, 2 + index * 0.02, True)
        self.assertEqual(pilot.throttle, 1)
        for index in range(150):
            pilot.update({'S'}, 5 + index * 0.02, True)
        self.assertEqual(pilot.throttle, 0)


if __name__ == '__main__':
    unittest.main()
