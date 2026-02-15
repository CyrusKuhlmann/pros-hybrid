"""Device models for simulation."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from vex_simulator.protocol import BrakeMode

if TYPE_CHECKING:
    pass


class Motor:
    """Simulated V5 smart motor — perfect-world (instant response).

    Assumes a perfect world: motor commands reach full speed instantly
    with no ramp-up, and any brake command immediately stops the motor.
    """

    MAX_CMD: int = 127

    def __init__(self, port: int, max_rpm: float = 200.0) -> None:
        """Initialize motor on *port* with the given *max_rpm*."""
        self.port = port
        self.max_rpm = max_rpm
        self.free_speed: float = max_rpm * math.pi / 30.0  # rad/s
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
        """Set brake behaviour for when voltage is zero.

        In perfect‑world mode all brake types behave identically:
        the motor stops instantly and velocity goes to zero.
        """
        self.brake_mode = mode

    def tare_position(self) -> None:
        """Reset encoder to zero."""
        self.position_deg = 0.0

    # ── physics interface (called by RobotState) ─────────────────

    def get_target_speed(self) -> float:
        """Return the ideal shaft speed [rad/s] for the current command.

        Perfect world: speed is an instant linear function of voltage.
        When voltage is zero (any brake mode) speed is immediately zero.
        """
        if self.voltage_cmd == 0:
            return 0.0
        return (self.voltage_cmd / self.MAX_CMD) * self.free_speed

    def update_state(self, shaft_rads: float, dt: float) -> None:
        """Write sensor readings from physics‑engine results."""
        self.velocity_rpm = shaft_rads * 30.0 / math.pi
        self.position_deg += math.degrees(shaft_rads) * dt
        self.torque_nm = 0.0  # perfect world — torque not modelled
        self.current_ma = 0
        self.temperature_c = 25.0  # no thermal effects

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
