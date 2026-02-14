# PROS Simulator

Compile a PROS C++ robot program on a regular desktop and run it against a Python physics simulator over TCP.

## Prerequisites

- **C++ toolchain**: GCC/MinGW with C++20 support (tested with MSYS2 UCRT64)
- **CMake** ≥ 3.16
- **Ninja** (or another CMake generator)
- **Python** ≥ 3.12
- **uv** (Python package manager) — or pip/venv

## Project Layout

```
pros-hardware/
├── pros-sim/          ← C++ PROS stubs + SimClient (this project)
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── pros/      ← Original PROS headers (unmodified)
│   │   └── sim/       ← SimClient header
│   └── src/
│       ├── pros/      ← Stub implementations that talk to the simulator
│       ├── sim/       ← TCP client (sim_client.cpp)
│       └── main.cpp   ← Entry point
└── vex-simulator/     ← Python physics simulator
    ├── pyproject.toml
    └── vex_simulator/
        ├── __main__.py
        ├── server.py
        ├── devices.py
        ├── robot_state.py
        └── protocol.py
```

## 1. Build the C++ Program

From the `pros-sim/` directory:

```bash
# Create and enter build directory
mkdir -p build
cd build

# Configure (Ninja generator recommended)
cmake .. -G Ninja

# Build
ninja
```

This produces `hello-world.exe` (Windows) or `hello-world` (Linux/macOS).

## 2. Start the Python Simulator

From the `vex-simulator/` directory:

```bash
# First time: create venv and install dependencies
uv sync

# Run the simulator
uv run python -m vex_simulator
```

Or with pip/venv:

```bash
python -m venv .venv
.venv\Scripts\activate      # Windows
# source .venv/bin/activate  # Linux/macOS
pip install -e .
python -m vex_simulator
```

The simulator starts a TCP server on `127.0.0.1:9090` and prints:

```
Simulator running on 127.0.0.1:9090
Waiting for client connections...
```

## 3. Run the C++ Program

With the simulator running, open a **second terminal** and run:

```bash
# From pros-sim/build/
./hello-world        # Linux/macOS
.\hello-world.exe    # Windows
```

The C++ program connects to the simulator, then calls `initialize()` and `opcontrol()` from `main.cpp`. You'll see motor commands in the simulator terminal and telemetry printed in the C++ terminal.

## How It Works

1. **Python simulator** runs a TCP server that models V5 devices (motors, IMU, rotation sensors, distance sensors, controller) with simple physics.
2. **C++ SimClient** (singleton) connects over TCP and communicates using newline-delimited JSON messages.
3. **PROS stub functions** (e.g. `motor_move()`, `imu_get_heading()`) call `SimClient` to send commands or read cached state instead of talking to real hardware.
4. The simulator broadcasts the full robot state at **50 Hz**. The C++ client caches it so telemetry reads are instant.

### Message Flow

```
C++ Program                          Python Simulator
    │                                       │
    │──── {"type":"handshake"} ────────────►│
    │◄─── {"type":"handshake","version":…} ─│
    │                                       │
    │──── {"type":"motor_move", …} ────────►│  (commands)
    │──── {"type":"motor_move_velocity",…}─►│
    │                                       │
    │◄─── {"type":"state_update", …} ───────│  (50 Hz broadcast)
    │◄─── {"type":"state_update", …} ───────│
    │                                       │
    │──── {"type":"disconnect"} ───────────►│
```

## Supported Commands

| Command | Direction | Description |
|---------|-----------|-------------|
| `motor_move` | C++ → Sim | Set motor voltage (-127 to 127) |
| `motor_move_velocity` | C++ → Sim | Set target velocity (RPM) |
| `motor_move_absolute` | C++ → Sim | Move to absolute position |
| `motor_move_relative` | C++ → Sim | Move relative distance |
| `motor_brake` | C++ → Sim | Apply current brake mode |
| `motor_set_brake_mode` | C++ → Sim | Set brake mode (coast/brake/hold) |
| `motor_tare_position` | C++ → Sim | Reset motor encoder to zero |
| `imu_reset` | C++ → Sim | Reset IMU heading/rotation |
| `rotation_reset` | C++ → Sim | Reset rotation sensor position |
| `rotation_set_position` | C++ → Sim | Set rotation sensor position |
| `state_update` | Sim → C++ | Full device state (50 Hz) |

## Troubleshooting

- **Connection refused**: Make sure the Python simulator is running before launching the C++ program.
- **Build errors on Windows**: Ensure you're using an MSYS2/MinGW toolchain (not MSVC) and that `cmake` / `ninja` are on your PATH.
- **Missing `ws2_32`**: The CMakeLists.txt links Winsock automatically on Windows. If you see linker errors, verify the `if(WIN32)` block is present.
- **Port already in use**: Kill any existing simulator process, or change the port in both `server.py` (`SimulatorServer.__init__`) and `sim_client.h` (`SimClient::connect()`).
