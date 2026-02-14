"""Protocol message definitions for client-simulator communication."""

from __future__ import annotations

from enum import Enum
from typing import Any


class MessageType(str, Enum):
    """Message types for simulator protocol."""

    # Commands (client → simulator)
    SET_MOTOR_VOLTAGE = "motor_move"
    SET_MOTOR_VELOCITY = "motor_move_velocity"
    SET_MOTOR_ABSOLUTE = "motor_move_absolute"
    SET_MOTOR_RELATIVE = "motor_move_relative"
    MOTOR_BRAKE = "motor_brake"
    SET_MOTOR_BRAKE_MODE = "motor_set_brake_mode"
    MOTOR_TARE_POSITION = "motor_tare_position"
    RESET_IMU = "imu_reset"
    RESET_ROTATION = "rotation_reset"
    SET_ROTATION_POSITION = "rotation_set_position"

    # State update (simulator → client, periodic)
    STATE_UPDATE = "state_update"

    # Lifecycle
    HANDSHAKE = "handshake"
    DISCONNECT = "disconnect"


class BrakeMode(str, Enum):
    """Motor brake modes."""

    COAST = "coast"
    BRAKE = "brake"
    HOLD = "hold"


class CommandMessage:
    """Base class for command messages from client."""

    def __init__(self, msg_type: MessageType, port: int, **kwargs: Any) -> None:
        """Initialize command message."""
        self.type = msg_type
        self.port = port
        self.params = kwargs

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CommandMessage:
        """Parse command from JSON dict."""
        msg_type = MessageType(data["type"])
        port = data["port"]
        return cls(
            msg_type,
            port,
            **{k: v for k, v in data.items() if k not in ("type", "port")},
        )

    def to_dict(self) -> dict[str, Any]:
        """Convert to JSON dict."""
        return {"type": self.type.value, "port": self.port, **self.params}


class StateUpdate:
    """State update message from simulator to client."""

    def __init__(
        self,
        timestamp_ms: int,
        motors: dict[int, dict[str, float]],
        imus: dict[int, dict[str, float]],
        rotation_sensors: dict[int, dict[str, float]],
        distance_sensors: dict[int, dict[str, float]],
        controller: dict[str, dict[str, int]],
    ) -> None:
        """Initialize state update."""
        self.timestamp_ms = timestamp_ms
        self.motors = motors
        self.imus = imus
        self.rotation_sensors = rotation_sensors
        self.distance_sensors = distance_sensors
        self.controller = controller

    def to_dict(self) -> dict[str, Any]:
        """Convert to JSON dict."""
        return {
            "type": MessageType.STATE_UPDATE.value,
            "timestamp_ms": self.timestamp_ms,
            "motors": self.motors,
            "imus": self.imus,
            "rotation": self.rotation_sensors,
            "distance": self.distance_sensors,
            "controller": self.controller,
        }
