"""Supervisor position feedback -> bounded RC; Betaflight owns inner control."""

import math


def clamp(value, low, high):
    return max(low, min(high, value))


def rate_stick(rate_deg_s):
    # Pinned default Actual rates: 70*s + 600*s*abs(s), zero expo.
    magnitude = (-70 + math.sqrt(70 ** 2 + 2400 * abs(rate_deg_s))) / 1200
    return math.copysign(clamp(magnitude, 0, 1), rate_deg_s)


class AutoTakeoff:
    def __init__(self):
        self.phase = 'waiting_ready'
        self.armed = False
        self.throttle = 0.0
        self.stable_since = None
        self.channels = [1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 10
        self.target = None
        self.yaw_target = 0.0
        self.arm_time = None
        self.takeoff_time = None
        self.takeoff_z = None
        self.z_integral = 0.0
        self.settled_since = None
        self.fault_reason = None

    def disarm(self, reason='motor_feedback_lost'):
        if self.phase not in ('waiting_ready', 'disarmed'):
            self.phase = 'fault'
            self.fault_reason = reason
        self.armed = False
        self.throttle = 0.0
        self.channels[2] = self.channels[4] = 1000

    def update(self, keys, now, connected, status, position, velocity, rotation, rates, dt):
        if 'SPACE' in keys:
            self.disarm()
            self.phase = 'disarmed'
            return self.channels
        if self.phase in ('fault', 'disarmed'):
            if 'R' in keys and position[2] < 0.2 and status and not status[0]:
                self.__init__()
            return self.channels
        if not connected or status is None:
            self.stable_since = None
            if self.armed:
                self.disarm('motor_feedback_lost' if not connected else 'msp_status_timeout')
            return self.channels
        if self.stable_since is None:
            self.stable_since = now
        confirmed, flags, _ = status
        if self.phase == 'waiting_ready':
            if now - self.stable_since < 2 or confirmed or flags:
                return self.channels
            self.target = [position[0], position[1], 3.0]
            self.yaw_target = math.atan2(rotation[3], rotation[0])
            self.armed = True
            self.channels[4] = 2000
            self.arm_time = now
            self.phase = 'arming'
            return self.channels
        if self.phase == 'arming':
            if confirmed:
                self.takeoff_time = now + 0.5
                self.takeoff_z = position[2]
                self.phase = 'taking_off'
            elif flags or now - self.arm_time > 5:
                self.disarm(f'arming_failed:{flags}')
            return self.channels
        if not confirmed:
            self.disarm(f'firmware_disarmed:{flags}')
            return self.channels
        if rotation[8] < math.cos(math.radians(45)):
            self.disarm('excessive_tilt')
            return self.channels
        if now < self.takeoff_time:
            return self.channels
        if self.phase == 'holding':
            self.target[0] = clamp(self.target[0] + 0.5 * dt * (('RIGHT' in keys) - ('LEFT' in keys)), -10, 10)
            self.target[1] = clamp(self.target[1] + 0.5 * dt * (('UP' in keys) - ('DOWN' in keys)), -10, 10)
            self.target[2] = clamp(self.target[2] + 0.4 * dt * (('W' in keys) - ('S' in keys)), 0.2, 10)
            self.yaw_target += 0.4 * dt * (('A' in keys) - ('D' in keys))
        target_z = min(self.target[2], self.takeoff_z + 0.6 * (now - self.takeoff_time)) if self.phase == 'taking_off' else self.target[2]
        ax = clamp(1.0 * (self.target[0] - position[0]) - 1.8 * velocity[0], -2, 2)
        ay = clamp(1.0 * (self.target[1] - position[1]) - 1.8 * velocity[1], -2, 2)
        self.z_integral = clamp(self.z_integral + (target_z - position[2]) * dt, -2, 2)
        az = clamp(2.0 * (target_z - position[2]) - 2.2 * velocity[2] + 0.3 * self.z_integral, -3, 3)
        yaw = math.atan2(rotation[3], rotation[0])
        forward = math.cos(yaw) * ax + math.sin(yaw) * ay
        left = -math.sin(yaw) * ax + math.cos(yaw) * ay
        roll = clamp(-math.atan2(left, 9.81 + az), -0.2, 0.2)
        pitch = clamp(math.atan2(forward, 9.81 + az), -0.2, 0.2)
        thrust = 1.5 * (9.81 + az) / (math.cos(roll) * math.cos(pitch))
        speed = math.sqrt(clamp(thrust / (4 * 0.0012 * 100 ** 2), 0, 1))
        # Pinned analog mixer: idle 5.5%, mincheck 1050, linear throttle curve.
        self.throttle = clamp(0.05 + 0.95 * (speed - 0.055) / 0.945, 0, 0.8)
        yaw_error = (self.yaw_target - yaw + math.pi) % (2 * math.pi) - math.pi
        yaw_rate = clamp(1.0 * yaw_error - 0.3 * rates[2], -0.5, 0.5)
        sticks = [rate_stick(math.degrees(roll) / 60 * 670),
                  rate_stick(math.degrees(pitch) / 60 * 670),
                  rate_stick(math.degrees(yaw_rate))]
        centered = lambda value: round(1500 + 500 * value + (math.copysign(5, value) if abs(value) > 0.001 else 0))
        self.channels = [centered(sticks[0]), centered(sticks[1]), round(1000 + 1000 * self.throttle),
                         centered(sticks[2]), 2000, 2000] + [1000] * 10
        if self.phase == 'taking_off' and target_z >= 3 and abs(position[2] - 3) < 0.15 and abs(velocity[2]) < 0.2:
            if self.settled_since is None:
                self.settled_since = now
            elif now - self.settled_since >= 1:
                self.phase = 'holding'
        else:
            self.settled_since = None
        return self.channels
