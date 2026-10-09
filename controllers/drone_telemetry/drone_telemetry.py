"""Publish sensor/truth telemetry and optionally run the Betaflight motor loop."""

import json
import math
import os
import sys
import time

from betaflight_bridge import BetaflightBridge, KeyboardPilot, StatusMonitor, MOTOR_NAMES, TIMEOUT_S
from auto_takeoff import AutoTakeoff


def pressure_pa(altitude_m):
    # shortcut: standard atmosphere below 11 km, add atmospheric layers for higher flights.
    if not math.isfinite(altitude_m) or altitude_m >= 11000:
        raise ValueError("Barometer altitude must be finite and below 11000 m")
    return 101325.0 * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588


def main():
    from controller import Supervisor

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    acceleration = robot.getDevice("accelerometer")
    gyro = robot.getDevice("gyro")
    acceleration.enable(timestep)
    gyro.enable(timestep)
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    bridge = None
    monitor = None
    automatic = os.environ.get('WEBOTS_AUTO_TAKEOFF') == '1'
    pilot = AutoTakeoff() if automatic else KeyboardPilot()
    keyboard = robot.getKeyboard()
    key_names = {keyboard.UP: "UP", keyboard.DOWN: "DOWN", keyboard.LEFT: "LEFT",
                 keyboard.RIGHT: "RIGHT", ord(" "): "SPACE"}
    keys = set()
    motor_velocities = [0.0] * 4
    try:
        if os.environ.get("WEBOTS_BETAFLIGHT_SITL") == "1":
            bridge = BetaflightBridge()
            bridge.start_rc()
            monitor = StatusMonitor()
            keyboard.enable(timestep)
            if automatic:
                print("Auto: takeoff to 3 m, then hold; arrows XY, W/S altitude, A/D yaw, Space abort", flush=True)
            else:
                print("SITL: R arm, Space disarm, W/S throttle, arrows roll/pitch, A/D yaw", flush=True)
        else:
            print("Iris telemetry only; motors stopped", flush=True)
        node = robot.getSelf()
        next_publish = 0.1
        next_rc = 0.0
        previous_rc = robot.getTime()
        previous_phase = None
        while robot.step(timestep) != -1:
            timestamp = robot.getTime()
            position = node.getPosition()
            rates = gyro.getValues()
            specific_force = acceleration.getValues()
            pressure = pressure_pa(position[2])
            age = None
            outputs = (0.0,) * 4
            applied = (0.0,) * 4
            status = None
            if bridge is not None:
                now = time.monotonic()
                bridge.send_sensors(timestamp, rates, specific_force, pressure)
                outputs, age = bridge.read_motors(now)
                connected = age is not None and age <= TIMEOUT_S
                status = monitor.state(now)
                if not connected:
                    pilot.disarm()
                    pilot.stable_since = None
                while (key := keyboard.getKey()) != -1:
                    key &= 0xffff
                    keys.add(key_names.get(key, chr(key).upper()))
                if "SPACE" in keys:
                    pilot.disarm()
                    bridge.send_rc(timestamp, pilot.channels)
                if timestamp >= next_rc:
                    if automatic:
                        channels = pilot.update(keys, now, connected, status, position,
                                                node.getVelocity()[:3], node.getOrientation(), rates,
                                                timestamp - previous_rc)
                        if pilot.phase != previous_phase:
                            print(f'Auto takeoff: {pilot.phase} {pilot.fault_reason or ""}', flush=True)
                            previous_phase = pilot.phase
                    else:
                        ready = status is not None and status[1] == 0
                        if pilot.armed and (status is None or (not status[0] and status[1])):
                            pilot.disarm()
                        channels = pilot.update(keys, now, connected, ready=ready)
                        if status is None or not status[0]:
                            channels[2] = 1000
                    bridge.send_rc(timestamp, channels)
                    previous_rc = timestamp
                    keys.clear()
                    next_rc = timestamp + 0.02
                if pilot.armed and connected and status is not None and status[0]:
                    desired = tuple(value * motor.getMaxVelocity() for value, motor in zip(outputs, motors))
                    # Motor response matches the reference plant's 50 ms actuator lag.
                    alpha = 1 - math.exp(-timestep / 1000 / 0.05)
                    motor_velocities = [actual + alpha * (target - actual)
                                        for actual, target in zip(motor_velocities, desired)]
                    applied = tuple(motor_velocities)
                else:
                    motor_velocities = [0.0] * 4
                for motor, velocity in zip(motors, applied):
                    motor.setVelocity(velocity)
            if timestamp < next_publish:
                continue
            telemetry = {
                "time_s": timestamp,
                "imu": {
                    "acceleration_body_m_s2": specific_force,
                    "angular_velocity_body_rad_s": rates,
                },
                "barometer": {"pressure_pa": pressure},
                "ground_truth": {
                    "position_enu_m": position,
                    "orientation_body_to_enu": node.getOrientation(),
                },
                "sitl": {
                    "enabled": bridge is not None,
                    "connected": age is not None and age <= TIMEOUT_S,
                    "motor_age_s": age,
                    "arm_requested": pilot.armed,
                    "armed": status[0] if status else False,
                    "arming_disable_flags": status[1] if status else None,
                    "auto_phase": pilot.phase if automatic else None,
                    "auto_fault": pilot.fault_reason if automatic else None,
                    "position_target_enu_m": pilot.target if automatic else None,
                    "rc_channels_us": pilot.channels,
                    "motor_outputs": outputs,
                    "motor_names": MOTOR_NAMES,
                    "applied_motor_commands_rad_s": applied,
                },
            }
            robot.setCustomData(json.dumps(telemetry, allow_nan=False))
            next_publish = timestamp + 0.1
    finally:
        for motor in motors:
            motor.setVelocity(0.0)
        if bridge is not None:
            try:
                pilot.disarm()
                bridge.send_rc(robot.getTime(), pilot.channels)
            finally:
                bridge.close()
        if monitor is not None:
            monitor.close()


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        assert pressure_pa(0) == 101325.0
        assert 0 < pressure_pa(1000) < pressure_pa(100) < pressure_pa(0)
        for invalid in (float("nan"), float("inf"), 11000):
            try:
                pressure_pa(invalid)
            except ValueError:
                pass
            else:
                raise AssertionError("Invalid altitude accepted")
        print("Barometer self-check passed")
    else:
        main()
