"""Launch automatic takeoff to 3 metres, then hold for Webots keyboard commands."""

import signal
import sys
import subprocess

from sitl import launch


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, interrupted)
    try:
        launch(auto_takeoff=True)
    except KeyboardInterrupt:
        sys.exit(130)
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f'Takeoff: {exc}', file=sys.stderr)
        sys.exit(1)
