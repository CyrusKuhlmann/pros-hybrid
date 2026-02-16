"""Robot state management."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

from vex_simulator.config import DeadWheelMount, RobotConfig, SensorMount
from vex_simulator.devices import Controller, DistanceSensor, IMU, Motor, RotationSensor
from vex_simulator.protocol import BrakeMode, CommandMessage, MessageType, StateUpdate

if TYPE_CHECKING:
    pass


class RobotState:
    """Central robot state — perfect‑world physics (instant response)."""

    IN_PER_M: float = 39.3701

    def __init__(self, config: RobotConfig | None = None) -> None:
        """Initialize robot from *config* (defaults used if omitted)."""
        self.cfg = config or RobotConfig()

        self.motors: dict[int, Motor] = {}
        self.imus: dict[int, IMU] = {}
        self.rotation_sensors: dict[int, RotationSensor] = {}
        self._dead_wheel_mounts: dict[int, DeadWheelMount] = {}
        self.distance_sensors: dict[int, DistanceSensor] = {}
        self.controller_master = Controller()
        self.controller_partner = Controller()

        self.start_time = time.time()

        # Ports listed as negative in config are reversed (voltage negated)
        self._reversed_ports: set[int] = set()
        for p in self.cfg.left_motor_ports + self.cfg.right_motor_ports:
            if p < 0:
                self._reversed_ports.add(abs(p))

        # Rotation sensor ports listed as negative are reversed (value negated)
        self._reversed_rotation_ports: set[int] = set()

        # Pre-create rotation sensors from config dead-wheel mounts
        for mount in self.cfg.rotation_sensors:
            abs_port = abs(mount.port)
            if mount.port < 0:
                self._reversed_rotation_ports.add(abs_port)
            self._dead_wheel_mounts[abs_port] = mount
            self.rotation_sensors[abs_port] = RotationSensor(abs_port)

        # Pose: +X=east, +Y=north, heading 0°=north, CW positive
        self.x: float = self.cfg.start_x_in
        self.y: float = self.cfg.start_y_in
        import math as _m

        self.heading_rad: float = _m.radians(self.cfg.start_heading_deg)

        # Velocities (SI, CW positive for angular)
        self.linear_vel: float = 0.0
        self.angular_vel: float = 0.0

        # Computed wall distances (mm)
        self.dist_front_mm: float = 2000.0
        self.dist_right_mm: float = 2000.0
        self.dist_left_mm: float = 2000.0

    def get_or_create_motor(self, port: int) -> Motor:
        """Get motor or create if doesn't exist."""
        if port not in self.motors:
            self.motors[port] = Motor(port, max_rpm=self.cfg.motor_max_rpm)
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
            if cmd.port in self._reversed_ports:
                voltage = -voltage
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
        """Advance physics by *dt* seconds using a force/torque model.

        Instead of instantly setting velocity from motor commands, motors
        produce torque according to their torque-speed curve.  That torque
        is converted to a net force/moment on the chassis, then integrated
        via F=ma to update velocity and position.  This naturally produces
        realistic acceleration, deceleration, and prevents the instant
        direction-reversal flicker seen with the old instant model.
        """
        R = self.cfg.wheel_radius_m
        half_track = self.cfg.track_width_m / 2
        mass = self.cfg.robot_mass_kg
        moi = self.cfg.robot_moi_kg_m2

        # -- derive current per-side shaft speeds from chassis velocity --
        # CW-positive angular_vel: left wheel is outer (faster), right is inner (slower)
        omega_left_shaft = (self.linear_vel + self.angular_vel * half_track) / R
        omega_right_shaft = (self.linear_vel - self.angular_vel * half_track) / R

        # -- collect motors per side (abs because negative = reversed) --
        left_motors = [
            self.motors[abs(p)]
            for p in self.cfg.left_motor_ports
            if abs(p) in self.motors
        ]
        right_motors = [
            self.motors[abs(p)]
            for p in self.cfg.right_motor_ports
            if abs(p) in self.motors
        ]

        # -- sum motor torques per side --
        tau_left = sum(m.get_torque(omega_left_shaft) for m in left_motors)
        tau_right = sum(m.get_torque(omega_right_shaft) for m in right_motors)

        # -- convert wheel torques to chassis force & moment --
        # Force at wheel contact = torque / radius
        f_left = tau_left / R
        f_right = tau_right / R

        net_force = f_left + f_right  # forward force on chassis [N]
        net_moment = (f_left - f_right) * half_track  # CW moment [N·m]

        # -- rolling friction (opposes current velocity) --
        g = 9.81
        friction_force = self.cfg.rolling_friction_coeff * mass * g
        if abs(self.linear_vel) > 1e-4:
            net_force -= math.copysign(friction_force, self.linear_vel)
        friction_moment = self.cfg.rolling_friction_coeff * mass * g * half_track
        if abs(self.angular_vel) > 1e-4:
            net_moment -= math.copysign(friction_moment, self.angular_vel)

        # -- integrate: F=ma  →  Δv = (F/m)·dt --
        linear_accel = net_force / mass
        angular_accel = net_moment / moi

        self.linear_vel += linear_accel * dt
        self.angular_vel += angular_accel * dt

        # -- prevent friction from reversing velocity (static friction clamp) --
        if abs(self.linear_vel) < 1e-4 and abs(net_force) < friction_force:
            self.linear_vel = 0.0
        if abs(self.angular_vel) < 1e-3 and abs(net_moment) < friction_moment:
            self.angular_vel = 0.0

        # -- integrate pose --
        self.heading_rad += self.angular_vel * dt
        self.x += self.linear_vel * math.sin(self.heading_rad) * dt * self.IN_PER_M
        self.y += self.linear_vel * math.cos(self.heading_rad) * dt * self.IN_PER_M

        # -- wall collisions (clamp position, project velocity) --
        half_robot = self.cfg.half_robot_in
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

        # -- update per-side shaft speeds after physics step --
        omega_left_actual = (self.linear_vel + self.angular_vel * half_track) / R
        omega_right_actual = (self.linear_vel - self.angular_vel * half_track) / R

        # -- update motor sensor states --
        for m in left_motors:
            m.update_state(omega_left_actual, dt)
        for m in right_motors:
            m.update_state(omega_right_actual, dt)

        # ── IMU ──
        angular_vel_dps = math.degrees(self.angular_vel)
        for imu in self.imus.values():
            imu.update(dt, angular_vel_dps)

        # ── rotation sensors (dead wheels) ──
        for port, rotation in self.rotation_sensors.items():
            mount = self._dead_wheel_mounts.get(port)
            if mount is None:
                continue
            if mount.orientation == "forward":
                ground_vel = self.linear_vel - self.angular_vel * (
                    mount.offset_right * 0.0254
                )
            else:  # "lateral"
                ground_vel = self.angular_vel * (mount.offset_forward * 0.0254)
            # Negative port in config → reversed sensor value
            if port in self._reversed_rotation_ports:
                ground_vel = -ground_vel
            rotation.update(dt, ground_vel, mount.wheel_radius_m)

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

        # Use configured sensor mounts, or fall back to defaults
        if self.cfg.distance_sensors:
            for mount in self.cfg.distance_sensors:
                d_mm = _dist(
                    mount.offset_forward,
                    -mount.offset_right,
                    mount._dx,
                    mount._dy,
                )
                if mount.port in self.distance_sensors:
                    self.distance_sensors[mount.port].distance_mm = int(d_mm)
                # Also keep convenience attrs for the first three
                if mount.direction == "forward":
                    self.dist_front_mm = d_mm
                elif mount.direction == "right":
                    self.dist_right_mm = d_mm
                elif mount.direction == "left":
                    self.dist_left_mm = d_mm
        else:
            hs = self.cfg.half_robot_in
            self.dist_front_mm = _dist(hs, 0.0, 1.0, 0.0)
            self.dist_right_mm = _dist(0.0, -hs, 0.0, -1.0)
            self.dist_left_mm = _dist(0.0, hs, 0.0, 1.0)
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
