"""Runtime preservation and occupied-port behavior."""

from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import sitl


class ToolTests(unittest.TestCase):
    def test_provision_preserves_existing_eeprom(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            eeprom = runtime / 'eeprom.bin'
            eeprom.write_bytes(b'keep settings')
            with patch.object(sitl, 'RUNTIME', runtime), patch.object(sitl, 'require_binary'):
                with self.assertRaisesRegex(RuntimeError, 'preserved'):
                    sitl.provision()
            self.assertEqual(eeprom.read_bytes(), b'keep settings')

    def test_busy_port_reports_conflict(self):
        with patch.object(sitl.socket, 'socket') as factory:
            factory.return_value.__enter__.return_value.bind.side_effect = OSError('occupied')
            with self.assertRaisesRegex(RuntimeError, 'UDP 9003 is occupied'):
                sitl.free_ports()

    def test_launcher_cleans_up_only_its_process_groups(self):
        with patch.object(sitl, 'require_runtime'), patch.object(sitl, 'free_ports'), \
                patch.object(sitl.subprocess, 'Popen') as spawn, patch.object(sitl.os, 'killpg') as kill:
            first, second = unittest.mock.MagicMock(), unittest.mock.MagicMock()
            first.pid, second.pid = 101, 102
            first.poll.return_value = second.poll.return_value = 0
            spawn.side_effect = [first, second]
            sitl.launch()
            self.assertTrue(all(call.kwargs['start_new_session'] for call in spawn.call_args_list))
            self.assertEqual(kill.call_args_list, [
                unittest.mock.call(102, sitl.signal.SIGTERM),
                unittest.mock.call(102, sitl.signal.SIGKILL),
                unittest.mock.call(101, sitl.signal.SIGTERM),
                unittest.mock.call(101, sitl.signal.SIGKILL),
            ])


if __name__ == '__main__':
    unittest.main()
