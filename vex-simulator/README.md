# VEX V5 Robot Simulator

Python-based robot simulator for PROS C++ client library testing.

## Setup

```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Create virtual environment and install dependencies
uv sync

# Run simulator
uv run python -m vex_simulator
```

## Architecture

- **TCP Server**: Listens on port 9090 for client connections
- **Physics Engine**: Simple Euler integration for motor/sensor simulation
- **State Broadcasting**: 50Hz state updates to connected clients
- **JSON Protocol**: Human-readable message format

## Supported Devices

- Motors (ports 1-21): voltage control, position/velocity/torque telemetry
- IMU (ports 1-21): heading, rotation, pitch, roll, yaw
- Rotation sensors (ports 1-21): position, velocity
- Distance sensors (ports 1-21): distance readings
- Controller: analog sticks, digital buttons
