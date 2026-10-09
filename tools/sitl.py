"""Build, provision, and launch the project-owned Betaflight runtime."""

import argparse
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / ".sitl/betaflight-source"
BINARY = ROOT / "bin/betaflight_SITL.elf"
RUNTIME = ROOT / ".sitl/runtime"
COMMIT = "e0b7bb01b17b21351057e9ead2d1ab39dd44fa16"
FLAGS = "-DSITL_EXTERNAL_BAROMETER -DSITL_WEBOTS_IMU_ONLY"


def free_ports(include_motor=True, include_webots=False):
    ports = [(socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004), (socket.SOCK_STREAM, 5761)]
    if include_motor:
        ports.append((socket.SOCK_DGRAM, 9002))
    if include_webots:
        ports.extend((socket.SOCK_STREAM, port) for port in (10022, 10023))
    for kind, port in ports:
        with socket.socket(socket.AF_INET, kind) as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError as exc:
                protocol = "UDP" if kind == socket.SOCK_DGRAM else "TCP"
                raise RuntimeError(f"{protocol} {port} is occupied. Stop the other simulation first.") from exc


def build():
    if not (SOURCE / ".git").exists():
        SOURCE.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "--depth", "1", "--branch", "2026.6.2",
                        "https://github.com/betaflight/betaflight.git", str(SOURCE)], check=True)
    actual = subprocess.check_output(["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip()
    if actual != COMMIT:
        raise RuntimeError(f"Expected firmware {COMMIT}, found {actual}; refusing to change this checkout")
    # Initialize before parallel make: simultaneous submodule hydration races on .git/config.
    subprocess.run(["git", "-C", str(SOURCE), "submodule", "update", "--init",
                    "src/config", "lib/modules/dronecan/libcanard"], check=True)
    for patch in sorted((ROOT / "tools/patches").glob("*.patch")):
        command = ["git", "-C", str(SOURCE), "apply"]
        if subprocess.run(command + ["--check", str(patch)], capture_output=True).returncode == 0:
            subprocess.run(command + [str(patch)], check=True)
        elif subprocess.run(command + ["--reverse", "--check", str(patch)], capture_output=True).returncode:
            raise RuntimeError(f"Firmware patch conflicts with local changes: {patch}")
    subprocess.run(["make", "-C", str(SOURCE), "TARGET=SITL", f"EXTRA_FLAGS={FLAGS}",
                    f"-j{min(os.cpu_count() or 1, 8)}"], check=True)
    BINARY.parent.mkdir(exist_ok=True)
    temporary = BINARY.with_suffix(".elf.tmp")
    shutil.copy2(SOURCE / "obj/main/betaflight_SITL.elf", temporary)
    temporary.chmod(0o755)
    temporary.replace(BINARY)
    print(f"Installed {BINARY}")


def require_binary():
    if not BINARY.is_file() or not os.access(BINARY, os.X_OK):
        raise RuntimeError("Missing bin/betaflight_SITL.elf. Run: python3 tools/sitl.py build")


def provision():
    require_binary()
    if (RUNTIME / "eeprom.bin").exists():
        raise RuntimeError("Existing .sitl/runtime/eeprom.bin preserved; provisioning refused")
    free_ports()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / "provision.log").open("w") as log:
        subprocess.run([str(BINARY), "--config", str(ROOT / "config/betaflight/webots.config")],
                       cwd=RUNTIME, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=30)
    log_text = (RUNTIME / "provision.log").read_text()
    if not (RUNTIME / "eeprom.bin").exists() or "###ERROR" in log_text:
        raise RuntimeError("Provisioning failed; inspect .sitl/runtime/provision.log")
    print(f"Provisioned {RUNTIME / 'eeprom.bin'}")


def require_runtime():
    require_binary()
    if not (RUNTIME / "eeprom.bin").is_file():
        raise RuntimeError("Missing runtime EEPROM. Run: python3 tools/sitl.py provision")


def launch():
    require_runtime()
    free_ports(include_webots=True)
    children = []
    try:
        children.append(subprocess.Popen([str(BINARY), "--ip", "127.0.0.1"], cwd=RUNTIME,
                                          start_new_session=True))
        env = os.environ.copy()
        env.update(WEBOTS_BETAFLIGHT_SITL="1", WEBOTS_EXTRA_PROJECT_PATH="/home/user/mcps/webotMCP")
        children.append(subprocess.Popen(["webots", "--batch", "--mode=realtime", "--stdout", "--stderr",
                                          str(ROOT / "worlds/drone.wbt")], cwd=ROOT, env=env,
                                         start_new_session=True))
        while all(child.poll() is None for child in children):
            time.sleep(0.2)
        for child in children:
            if child.poll() not in (None, 0):
                raise RuntimeError(f"Simulation process {child.pid} exited with {child.returncode}")
    finally:
        for child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            # Webots' shell launcher may exit before its simulator/controllers.
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("build", "provision", "run", "launch"))
    action = parser.parse_args().action
    if action == "run":
        require_runtime()
        free_ports(include_motor=False)
        os.chdir(RUNTIME)
        os.execv(BINARY, [str(BINARY), "--ip", "127.0.0.1"])
    {"build": build, "provision": provision, "launch": launch}[action]()


if __name__ == "__main__":
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"SITL: {exc}", file=sys.stderr)
        sys.exit(1)
