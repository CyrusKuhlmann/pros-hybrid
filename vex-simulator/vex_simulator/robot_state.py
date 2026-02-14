"""Robot state management."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

from vex_simulator.devices import Controller, DistanceSensor, IMU, Motor, RotationSensor
from vex_simulator.protocol import BrakeMode, CommandMessage, MessageType, StateUpdate

if TYPE_CHECKING:
    pass


class RobotState:
    """Central robot state — perfect‑world physics (instant response)."""

    # ── Robot physical constants ────────────────────────────────────
    WHEEL_RADIUS: float = 0.0508  # m   (2‑inch wheels)
    TRACKWIDTH: float = 0.3556  # m   (14‑inch track centre‑to‑centre)
    IN_PER_M: float = 39.3701
    LEFT_PORTS: tuple[int, ...] = (1, 2, 3)
    RIGHT_PORTS: tuple[int, ...] = (4, 5, 6)

    def __init__(self) -> None:
        """Initialize robot with empty device registry."""
        self.motors: dict[int, Motor] = {}
        self.imus: dict[int, IMU] = {}
        self.rotation_sensors: dict[int, RotationSensor] = {}
        self.distance_sensors: dict[int, DistanceSensor] = {}
        self.controller_master = Controller()
        self.controller_partner = Controller()

        self.start_time = time.time()

        # Pose (field coords, inches, origin bottom‑left)
        self.x: float = 81.0
        self.y: float = 27.0
        self.heading_rad: float = 0.0  # θ=0 → north / up, CCW positive

        # Rigid‑body velocities (SI)
        self.linear_vel: float = 0.0  # m/s  (forward)
        self.angular_vel: float = 0.0  # rad/s (CCW positive)

        # Computed wall distances (mm) for the three mounted sensors
        self.dist_front_mm: float = 2000.0
        self.dist_right_mm: float = 2000.0
        self.dist_left_mm: float = 2000.0

    def get_or_create_motor(self, port: int) -> Motor:
        """Get motor or create if doesn't exist."""
        if port not in self.motors:
            self.motors[port] = Motor(port)
        return self.motors[port]

    def get_or_create_imu(self, port: int) -> IMU:
        """Get IMU or create if doesn't exist."""
        if port not in self.imus:
            self.imus[port] = IMU(port)
        return self.imus[port]

    def get_or_create_rotation(self, port: int) -> RotationSensor:
        """Get rotation sensor or create if doesn't exist."""
        if port not in self.rotation_sensors:
            self.rotation_sensors[port] = RotationSensor(port)
        return self.rotation_sensors[port]

    def get_or_create_distance(self, port: int) -> DistanceSensor:
        """Get distance sensor or create if doesn't exist."""
        if port not in self.distance_sensors:
            self.distance_sensors[port] = DistanceSensor(port)
        return self.distance_sensors[port]

    def handle_command(self, cmd: CommandMessage) -> None:
        """Process a low‑level command from the client."""
        if cmd.type == MessageType.SET_MOTOR_VOLTAGE:
            motor = self.get_or_create_motor(cmd.port)
            voltage = cmd.params.get("voltage", 0)
            motor.set_voltage(voltage)

        elif cmd.type == MessageType.SET_MOTOR_BRAKE_MODE:
            motor = self.get_or_create_motor(cmd.port)
            mode = BrakeMode(cmd.params.get("mode", "coast"))
            motor.set_brake_mode(mode)

        elif cmd.type == MessageType.MOTOR_TARE_POSITION:
            motor = self.get_or_create_motor(cmd.port)
            motor.tare_position()

        elif cmd.type == MessageType.RESET_IMU:
            imu = self.get_or_create_imu(cmd.port)
            imu.reset()

        elif cmd.type == MessageType.RESET_ROTATION:
            rotation = self.get_or_create_rotation(cmd.port)
            rotation.reset()

        elif cmd.type == MessageType.SET_ROTATION_POSITION:
            rotation = self.get_or_create_rotation(cmd.port)
            position = cmd.params.get("position", 0)
            rotation.set_position(position)

    def update(self, dt: float) -> None:
        """Advance perfect-world physics by *dt* seconds.

        No momentum, no ramp-up: motor speeds are applied instantly.
        Any brake (coast/brake/hold) immediately zeroes motor velocity.
        """
        half_track = self.TRACKWIDTH / 2

        # -- collect motors per side --
        left_motors = [self.motors[p] for p in self.LEFT_PORTS if p in self.motors]
        right_motors = [self.motors[p] for p in self.RIGHT_PORTS if p in self.motors]

        # -- instant target shaft speeds from motor commands --
        if left_motors:
            omega_left = sum(m.get_target_speed() for m in left_motors) / len(left_motors)
        else:
            omega_left = 0.0
        if right_motors:
            omega_right = sum(m.get_target_speed() for m in right_motors) / len(right_motors)
        else:
            omega_right = 0.0

        # -- convert shaft speeds to ground velocities --
        v_left = omega_left * self.WHEEL_RADIUS
        v_right = omega_right * self.WHEEL_RADIUS

        # -- differential drive kinematics (instant, no inertia) --
        self.linear_vel = (v_left + v_right) / 2.0
        self.angular_vel = (v_right - v_left) / self.TRACKWIDTH

        # -- integrate pose (field inches, theta=0 -> north) --
        self.heading_rad += self.angular_vel * dt
        self.x += self.linear_vel * math.sin(self.heading_rad) * dt * self.IN_PER_M
        self.y += self.linear_vel * math.cos(self.heading_rad) * dt * self.IN_PER_M

        # -- wall collisions (clamp position, project velocity) --
        half_robot = 9.0  # robot is 18x18 in
        vx = self.linear_vel * math.sin(self.heading_rad)
        vy = self.linear_vel * math.cos(self.heading_rad)
        clamped = False
        if self.x < half_robot:
            self.x = half_robot
            if vx < 0:
                vx = 0.0
                clamped = True
        elif self.x > 144.0 - half_robot:
            self.x = 144.0 - half_robot
            if vx > 0:
                vx = 0.0
                clamped = True
        if self.y < half_robot:
            self.y = half_robot
            if vy < 0:
                vy = 0.0
                clamped = True
        elif self.y > 144.0 - half_robot:
            self.y = 144.0 - half_robot
            if vy > 0:
                vy = 0.0
                clamped = True
        if clamped:
            s = math.sin(self.heading_rad)
            c = math.cos(self.heading_rad)
            self.linear_vel = vx * s + vy * c

        # -- update motor sensor states --
        omega_left_actual = (self.linear_vel - self.angular_vel * half_track) / self.WHEEL_RADIUS
        omega_right_actual = (self.linear_vel + self.angular_vel * half_track) / self.WHEEL_RADIUS
        for m in left_motors:
            m.update_state(omega_left_actual, dt)
        for m in right_motors:
            m.update_state(omega_right_actual, dt)

        # ── IMU ──
        angular_vel_dps = math.degrees(self.angular_vel)
        for imu in self.imus.values():
            imu.update(dt, angular_vel_dps)

        # ── rotation sensors (coupled to same‑port motor) ──
        for port, rotation in self.rotation_sensors.items():
            motor_vel = self.motors[port].velocity_rpm if port in self.motors else 0.0
            rotation.update(dt, motor_vel)

        # ── distance sensors ──
        self._update_distance_sensors()

    # ── distance‑sensor raycasting ──────────────────────────────────────
    def _update_distance_sensors(self) -> None:
        """Compute distance to nearest wall for each mounted sensor."""
        cos_h = math.cos(self.heading_rad)
        sin_h = math.sin(self.heading_rad)

        def _dist(lx: float, ly: float, dx: float, dy: float) -> float:
            ox = self.x + lx * sin_h - ly * cos_h
            oy = self.y + lx * cos_h + ly * sin_h
            fdx = dx * sin_h - dy * cos_h
            fdy = dx * cos_h + dy * sin_h
            return self._raycast(ox, oy, fdx, fdy) * 25.4  # → mm

        self.dist_front_mm = _dist(9.0, 0.0, 1.0, 0.0)
        self.dist_right_mm = _dist(0.0, -9.0, 0.0, -1.0)
        self.dist_left_mm = _dist(0.0, 9.0, 0.0, 1.0)

        # Mirror into any port‑based DistanceSensor objects
        for sensor in self.distance_sensors.values():
            sensor.distance_mm = int(self.dist_front_mm)

    @staticmethod
    def _raycast(ox: float, oy: float, dx: float, dy: float) -> float:
        """Return distance (inches) from *ox,oy* along unit‑vector *dx,dy* to nearest field wall."""
        field = 144.0
        best = 9999.0
        if abs(dx) > 1e-10:
            for wall_x in (0.0, field):
                t = (wall_x - ox) / dx
                if t > 0.001:
                    yh = oy + t * dy
                    if -0.001 <= yh <= field + 0.001:
                        best = min(best, t)
        if abs(dy) > 1e-10:
            for wall_y in (0.0, field):
                t = (wall_y - oy) / dy
                if t > 0.001:
                    xh = ox + t * dx
                    if -0.001 <= xh <= field + 0.001:
                        best = min(best, t)
        return best

    def get_state_update(self) -> StateUpdate:
        """Generate state update message."""
        timestamp_ms = int((time.time() - self.start_time) * 1000)

        return StateUpdate(
            timestamp_ms=timestamp_ms,
            motors={port: motor.get_state() for port, motor in self.motors.items()},
            imus={port: imu.get_state() for port, imu in self.imus.items()},
            rotation_sensors={
                port: sensor.get_state()
                for port, sensor in self.rotation_sensors.items()
            },
            distance_sensors={
                port: sensor.get_state()
                for port, sensor in self.distance_sensors.items()
            },
            controller={
                "master": self.controller_master.get_state(),
                "partner": self.controller_partner.get_state(),
            },
        )
