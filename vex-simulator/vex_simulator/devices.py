"""Device models for simulation."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from vex_simulator.protocol import BrakeMode

if TYPE_CHECKING:
    pass


class Motor:
    """Simulated V5 motor."""

    # Motor specs (200 RPM green cartridge)
    MAX_VOLTAGE = 127
    MAX_RPM = 200.0
    GEAR_RATIO = 18.0  # 18:1 green cartridge

    def __init__(self, port: int) -> None:
        """Initialize motor."""
        self.port = port
        self.voltage = 0
        self.position_deg = 0.0
        self.velocity_rpm = 0.0
        self.torque_nm = 0.0
        self.temperature_c = 25.0
        self.current_ma = 0
        self.brake_mode = BrakeMode.COAST

    def set_voltage(self, voltage: int) -> None:
        """Set motor voltage (-127 to 127)."""
        self.voltage = max(-self.MAX_VOLTAGE, min(self.MAX_VOLTAGE, voltage))

    def set_brake_mode(self, mode: BrakeMode) -> None:
        """Set brake mode."""
        self.brake_mode = mode

    def update(self, dt: float) -> None:
        """Update motor state with physics simulation."""
        # Simple model: velocity proportional to voltage
        target_rpm = (self.voltage / self.MAX_VOLTAGE) * self.MAX_RPM

        # Simple acceleration model
        accel_rpm_per_sec = 600.0  # Reaches max speed in ~0.33 seconds
        if abs(target_rpm - self.velocity_rpm) < accel_rpm_per_sec * dt:
            self.velocity_rpm = target_rpm
        elif target_rpm > self.velocity_rpm:
            self.velocity_rpm += accel_rpm_per_sec * dt
        else:
            self.velocity_rpm -= accel_rpm_per_sec * dt

        # Apply brake mode when voltage is 0
        if self.voltage == 0:
            if self.brake_mode == BrakeMode.BRAKE:
                # Aggressive braking
                if abs(self.velocity_rpm) < 50.0 * dt:
                    self.velocity_rpm = 0.0
                else:
                    self.velocity_rpm *= 1.0 - 10.0 * dt
            elif self.brake_mode == BrakeMode.HOLD:
                # Active hold (stop completely)
                self.velocity_rpm = 0.0
            # COAST: no additional damping

        # Update position
        deg_per_sec = self.velocity_rpm * 6.0  # 360 deg / 60 sec
        self.position_deg += deg_per_sec * dt

        # Current draw proportional to voltage
        self.current_ma = int(abs(self.voltage) * 20)  # ~2.5A at full voltage

        # Temperature rises with use
        heat_rate = abs(self.voltage) * 0.01 * dt
        cooling_rate = (self.temperature_c - 25.0) * 0.1 * dt
        self.temperature_c += heat_rate - cooling_rate

        # Torque estimate
        self.torque_nm = (
            abs(self.voltage) / self.MAX_VOLTAGE * 1.05
        )  # ~1.05 Nm stall torque

    def get_state(self) -> dict[str, float]:
        """Get motor state as dict."""
        return {
            "position_deg": round(self.position_deg, 2),
            "velocity_rpm": round(self.velocity_rpm, 2),
            "torque_nm": round(self.torque_nm, 3),
            "temp_c": round(self.temperature_c, 1),
            "current_ma": self.current_ma,
            "voltage": self.voltage,
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
    """Simulated rotation sensor."""

    def __init__(self, port: int) -> None:
        """Initialize rotation sensor."""
        self.port = port
        self.position_cdeg = 0  # Centidegrees
        self.velocity_dps = 0.0  # Degrees per second

    def reset(self) -> None:
        """Reset position to zero."""
        self.position_cdeg = 0

    def update(self, dt: float, attached_motor_velocity_rpm: float) -> None:
        """Update sensor based on attached motor."""
        # Assume 1:1 coupling to a motor for now
        self.velocity_dps = attached_motor_velocity_rpm * 6.0  # Convert RPM to deg/s
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
