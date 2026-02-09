"""Robot state management."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from vex_simulator.devices import Controller, DistanceSensor, IMU, Motor, RotationSensor
from vex_simulator.protocol import BrakeMode, CommandMessage, MessageType, StateUpdate

if TYPE_CHECKING:
    pass


class RobotState:
    """Central robot state for simulation."""

    def __init__(self) -> None:
        """Initialize robot with empty device registry."""
        self.motors: dict[int, Motor] = {}
        self.imus: dict[int, IMU] = {}
        self.rotation_sensors: dict[int, RotationSensor] = {}
        self.distance_sensors: dict[int, DistanceSensor] = {}
        self.controller_master = Controller()
        self.controller_partner = Controller()

        self.start_time = time.time()

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
        """Process command from client."""
        if cmd.type == MessageType.SET_MOTOR_VOLTAGE:
            motor = self.get_or_create_motor(cmd.port)
            voltage = cmd.params.get("voltage", 0)
            motor.set_voltage(voltage)
            print(f"Motor {cmd.port}: voltage={voltage}")

        elif cmd.type == MessageType.SET_MOTOR_BRAKE_MODE:
            motor = self.get_or_create_motor(cmd.port)
            mode_str = cmd.params.get("mode", "coast")
            mode = BrakeMode(mode_str)
            motor.set_brake_mode(mode)
            print(f"Motor {cmd.port}: brake_mode={mode.value}")

        elif cmd.type == MessageType.RESET_IMU:
            imu = self.get_or_create_imu(cmd.port)
            imu.reset()
            print(f"IMU {cmd.port}: reset")

        elif cmd.type == MessageType.RESET_ROTATION:
            rotation = self.get_or_create_rotation(cmd.port)
            rotation.reset()
            print(f"Rotation {cmd.port}: reset")

    def update(self, dt: float) -> None:
        """Update all device physics."""
        # Update motors
        for motor in self.motors.values():
            motor.update(dt)

        # Calculate robot angular velocity from differential drive
        # Assume ports 1-3 are left, 4-6 are right (example configuration)
        left_velocity = sum(
            self.motors[p].velocity_rpm for p in [1, 2, 3] if p in self.motors
        )
        right_velocity = sum(
            self.motors[p].velocity_rpm for p in [4, 5, 6] if p in self.motors
        )

        # Simple differential: angular velocity proportional to difference
        # Assuming 12-inch wheelbase, this is a rough estimate
        wheelbase_inches = 12.0
        wheel_diameter_inches = 4.0
        angular_velocity_deg_per_sec = (
            (right_velocity - left_velocity)
            / 6.0
            * (wheel_diameter_inches / wheelbase_inches)
            * 10.0
        )

        # Update IMUs
        for imu in self.imus.values():
            imu.update(dt, angular_velocity_deg_per_sec)

        # Update rotation sensors (assume coupled to motors for now)
        for port, rotation in self.rotation_sensors.items():
            # Try to find corresponding motor
            motor_velocity = (
                self.motors[port].velocity_rpm if port in self.motors else 0.0
            )
            rotation.update(dt, motor_velocity)

        # Update distance sensors
        for distance in self.distance_sensors.values():
            distance.update(dt)

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
