"""Device models for simulation."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from vex_simulator.protocol import BrakeMode

if TYPE_CHECKING:
    pass


class Motor:
    """Simulated V5 smart motor — pure DC‑motor physics, no PID.

    Models a V5 motor with green cartridge (200 RPM, 18:1 internal gear).
    Only accepts raw voltage commands (−127…+127).  Shaft speed and
    position are determined by the rigid‑body physics engine in
    *RobotState*, not by the motor independently.
    """

    # ── V5 green‑cartridge constants ──────────────────────────────
    MAX_CMD: int = 127
    V_NOM: float = 12.0  # battery voltage [V]
    FREE_SPEED: float = 200.0 * math.pi / 30.0  # 200 RPM → rad/s ≈ 20.944
    STALL_TORQUE: float = 2.1  # Nm at output shaft

    def __init__(self, port: int) -> None:
        """Initialize motor on *port*."""
        self.port = port
        self.voltage_cmd: int = 0  # raw command −127…+127
        self.brake_mode: BrakeMode = BrakeMode.COAST
        self._hold_pos: float | None = None  # latched encoder for HOLD

        # Sensor state — written by the physics engine every tick
        self.velocity_rpm: float = 0.0
        self.position_deg: float = 0.0
        self.torque_nm: float = 0.0
        self.current_ma: int = 0
        self.temperature_c: float = 25.0

    # ── low‑level commands (the ONLY motor inputs) ─────────────────

    def set_voltage(self, voltage: int) -> None:
        """Set raw voltage (−127 … +127)."""
        prev = self.voltage_cmd
        self.voltage_cmd = max(-self.MAX_CMD, min(self.MAX_CMD, voltage))
        if self.voltage_cmd != 0:
            self._hold_pos = None
        elif prev != 0 and self.brake_mode == BrakeMode.HOLD:
            self._hold_pos = self.position_deg

    def set_brake_mode(self, mode: BrakeMode) -> None:
        """Set brake behaviour for when voltage is zero."""
        self.brake_mode = mode
        if mode == BrakeMode.HOLD and self.voltage_cmd == 0:
            self._hold_pos = self.position_deg

    def tare_position(self) -> None:
        """Reset encoder to zero."""
        self.position_deg = 0.0
        if self._hold_pos is not None:
            self._hold_pos = 0.0

    # ── physics interface (called by RobotState) ─────────────────

    def compute_torque(self, shaft_rads: float) -> float:
        """Return output‑shaft torque [Nm] for current voltage & shaft speed.

        DC motor model:  τ = τ_stall × (V_cmd/V_nom − ω/ω_free)

        Brake modes when voltage_cmd == 0:
          COAST — open circuit → τ = 0
          BRAKE — shorted terminals → τ = −τ_stall × ω/ω_free
          HOLD  — firmware spring+damper holding encoder position
        """
        if self.voltage_cmd == 0:
            if self.brake_mode == BrakeMode.COAST:
                return 0.0
            if self.brake_mode == BrakeMode.BRAKE:
                return -self.STALL_TORQUE * shaft_rads / self.FREE_SPEED
            # HOLD
            if self._hold_pos is None:
                self._hold_pos = self.position_deg
            err_rad = math.radians(self._hold_pos - self.position_deg)
            return 5.0 * err_rad - 3.0 * shaft_rads

        v_frac = self.voltage_cmd / self.MAX_CMD
        return self.STALL_TORQUE * (v_frac - shaft_rads / self.FREE_SPEED)

    def update_state(self, shaft_rads: float, torque: float, dt: float) -> None:
        """Write sensor readings from physics‑engine results."""
        self.velocity_rpm = shaft_rads * 30.0 / math.pi
        self.position_deg += math.degrees(shaft_rads) * dt
        self.torque_nm = torque
        self.current_ma = int(abs(torque) / self.STALL_TORQUE * 2500)
        # Simple thermal model (I²R heating, convective cooling)
        power_w = (self.current_ma * 0.001) ** 2 * 5.0
        self.temperature_c += (
            power_w * 0.002 - (self.temperature_c - 25.0) * 0.01
        ) * dt
        self.temperature_c = min(self.temperature_c, 55.0)

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
    """Simulated rotation sensor."""

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
