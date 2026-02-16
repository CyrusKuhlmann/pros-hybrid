"""Device models for simulation."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from vex_simulator.protocol import BrakeMode

if TYPE_CHECKING:
    pass


class Motor:
    """Simulated V5 smart motor with realistic torque-speed curve.

    Models the linear torque-speed relationship of a DC motor:
      torque = stall_torque * (voltage_fraction - speed / free_speed)

    A dead-zone below ~5% voltage prevents jitter from tiny commands.
    """

    MAX_CMD: int = 127

    # V5 motor constants (11W smart motor, 200 RPM green cartridge baseline)
    # Stall torque ~2.1 N·m for 200 RPM green cartridge.
    # Scaled proportionally for other cartridges via gear ratio.
    _BASE_RPM: float = 200.0
    _BASE_STALL_TORQUE: float = 2.1  # N·m at 200 RPM gearing

    # Dead-zone: commands below this fraction of 127 produce no torque.
    DEAD_ZONE_FRAC: float = 0.04  # ~5/127

    def __init__(self, port: int, max_rpm: float = 200.0) -> None:
        """Initialize motor on *port* with the given *max_rpm*."""
        self.port = port
        self.max_rpm = max_rpm
        self.free_speed: float = max_rpm * math.pi / 30.0  # rad/s
        # Stall torque scales inversely with gear ratio (higher RPM = less torque)
        self.stall_torque: float = self._BASE_STALL_TORQUE * (self._BASE_RPM / max_rpm)

        self.voltage_cmd: int = 0  # raw command −127…+127
        self.brake_mode: BrakeMode = BrakeMode.COAST

        # Sensor state — written by the physics engine every tick
        self.velocity_rpm: float = 0.0
        self.position_deg: float = 0.0
        self.torque_nm: float = 0.0
        self.current_ma: int = 0
        self.temperature_c: float = 25.0

    # ── low‑level commands (the ONLY motor inputs) ─────────────────

    def set_voltage(self, voltage: int) -> None:
        """Set raw voltage (−127 … +127)."""
        self.voltage_cmd = max(-self.MAX_CMD, min(self.MAX_CMD, voltage))

    def set_brake_mode(self, mode: BrakeMode) -> None:
        """Set brake behaviour for when voltage is zero."""
        self.brake_mode = mode

    def tare_position(self) -> None:
        """Reset encoder to zero."""
        self.position_deg = 0.0

    # ── physics interface (called by RobotState) ─────────────────

    def get_torque(self, shaft_speed_rads: float) -> float:
        """Return motor output torque [N·m] given current shaft speed.

        Uses the DC motor torque-speed curve:
          T = T_stall * (V_frac - ω / ω_free)
        with a dead-zone: if |V_frac| < DEAD_ZONE, T = 0.
        """
        v_frac = self.voltage_cmd / self.MAX_CMD

        # Dead-zone: tiny voltage commands produce no torque
        if abs(v_frac) < self.DEAD_ZONE_FRAC:
            # Brake / hold modes actively resist motion
            if self.brake_mode in (BrakeMode.BRAKE, BrakeMode.HOLD):
                return -self.stall_torque * (shaft_speed_rads / self.free_speed)
            return 0.0

        return self.stall_torque * (v_frac - shaft_speed_rads / self.free_speed)

    def update_state(self, shaft_rads: float, dt: float) -> None:
        """Write sensor readings from physics‑engine results."""
        self.velocity_rpm = shaft_rads * 30.0 / math.pi
        self.position_deg += math.degrees(shaft_rads) * dt

    def get_state(self) -> dict[str, float]:
        """Motor state dict sent to client."""
        return {
            "position_deg": round(self.position_deg, 2),
            "velocity_rpm": round(self.velocity_rpm, 2),
            "torque_nm": round(self.torque_nm, 3),
            "temp_c": round(self.temperature_c, 1),
            "current_ma": self.current_ma,
            "voltage": self.voltage_cmd,
        }


class IMU:
    """Simulated inertial measurement unit."""

    def __init__(self, port: int) -> None:
        """Initialize IMU."""
        self.port = port
        self.heading_deg = 0.0  # 0-360
        self.rotation_deg = 0.0  # Unbounded
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.yaw_deg = 0.0
        self.is_calibrating = False

    def reset(self) -> None:
        """Reset IMU heading/rotation to zero."""
        self.rotation_deg = 0.0
        self.heading_deg = 0.0

    def update(self, dt: float, robot_angular_velocity: float) -> None:
        """Update IMU based on robot rotation."""
        # Integrate angular velocity
        self.rotation_deg += robot_angular_velocity * dt
        self.heading_deg = self.rotation_deg % 360.0
        self.yaw_deg = self.heading_deg

        # For now, pitch and roll are 0 (robot on flat ground)
        self.pitch_deg = 0.0
        self.roll_deg = 0.0

    def get_state(self) -> dict[str, float]:
        """Get IMU state as dict."""
        return {
            "heading": round(self.heading_deg, 2),
            "rotation": round(self.rotation_deg, 2),
            "pitch": round(self.pitch_deg, 2),
            "roll": round(self.roll_deg, 2),
            "yaw": round(self.yaw_deg, 2),
        }


class RotationSensor:
    """Simulated rotation sensor attached to a dead wheel."""

    def __init__(self, port: int) -> None:
        """Initialize rotation sensor."""
        self.port = port
        self.position_cdeg = 0  # Centidegrees
        self.velocity_dps = 0.0  # Degrees per second

    def reset(self) -> None:
        """Reset position to zero."""
        self.position_cdeg = 0

    def set_position(self, position_cdeg: int) -> None:
        """Set the current position value."""
        self.position_cdeg = position_cdeg

    def update(
        self, dt: float, ground_velocity_ms: float, wheel_radius_m: float
    ) -> None:
        """Update sensor from the dead wheel's ground contact velocity.

        *ground_velocity_ms* is the linear speed (m/s) at the wheel's
        contact patch.  *wheel_radius_m* converts that to angular rate.
        """
        import math

        wheel_angular_vel = ground_velocity_ms / wheel_radius_m  # rad/s
        self.velocity_dps = math.degrees(wheel_angular_vel)
        self.position_cdeg += int(self.velocity_dps * dt * 100)  # Integrate

    def get_state(self) -> dict[str, float]:
        """Get rotation sensor state as dict."""
        return {
            "position_cdeg": self.position_cdeg,
            "velocity_dps": round(self.velocity_dps, 2),
        }


class DistanceSensor:
    """Simulated distance sensor."""

    def __init__(self, port: int) -> None:
        """Initialize distance sensor."""
        self.port = port
        self.distance_mm = 2000  # Default: 2 meters

    def update(self, dt: float) -> None:
        """Update sensor (could add object detection later)."""
        # For now, static distance
        pass

    def get_state(self) -> dict[str, float]:
        """Get distance sensor state as dict."""
        return {
            "distance_mm": self.distance_mm,
        }


class Controller:
    """Simulated VEX controller."""

    def __init__(self) -> None:
        """Initialize controller with zero inputs."""
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0
        self.buttons = 0  # Bitmask

    def get_state(self) -> dict[str, int]:
        """Get controller state as dict."""
        return {
            "left_x": self.left_x,
            "left_y": self.left_y,
            "right_x": self.right_x,
            "right_y": self.right_y,
            "buttons": self.buttons,
        }
