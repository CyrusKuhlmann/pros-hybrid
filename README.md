# PROS Hybrid — VEX V5 Robot Simulator

Test and iterate on your PROS C++ robot code **without a physical robot**. This project pairs a drop-in C++ stub library (`pros-sim`) with a Python physics simulator & visualizer (`vex-simulator`) so your existing `main.cpp` compiles on a desktop and runs against a simulated field over a local TCP connection.

![Python 3.12+](https://img.shields.io/badge/python-3.12%2B-blue) ![C++20](https://img.shields.io/badge/C%2B%2B-20-blue) ![License](https://img.shields.io/badge/license-MIT-green)

---

## Why Use This?

- **No robot required** — tune PID loops, test autonomous routines, and debug logic from your laptop.
- **Real PROS headers** — your code includes the same `pros/motors.hpp`, `pros/imu.hpp`, etc. No API changes needed.
- **Live visualizer** — see your robot drive on a to-scale VRC field with real-time telemetry (motor output, IMU heading, distance sensors, odometry trail).
- **Path editor** — visually design Catmull-Rom spline paths on the field and export them as C++ code.
- **Configurable robot** — define your drivetrain, sensors, and starting pose in a single `robot_config.toml` file.

---

## Architecture Overview

```
┌──────────────────────────┐         TCP (JSON, port 9090)         ┌──────────────────────────┐
│        pros-sim           │ ──────────────────────────────────── │      vex-simulator        │
│  (C++ desktop executable) │                                      │  (Python physics engine)  │
│                           │  motor_move, imu_reset, ...  ──────► │                           │
│  Your main.cpp            │                                      │  Euler-integrated physics │
│  + PROS stub library      │  ◄────── state_update (50 Hz)        │  + Pygame visualizer      │
└──────────────────────────┘                                       └──────────────────────────┘
```

1. **`pros-sim/`** — Your robot code (`main.cpp`, autonomous routines, PID controllers, etc.) compiled with stub implementations of every PROS API function. Each stub forwards commands to the simulator over TCP and reads back cached sensor state.
2. **`vex-simulator/`** — A Python package that runs a TCP server, simulates V5 motors/sensors with a torque-speed physics model, and renders a live Pygame window showing the 144 × 144 in. field.

---

## Supported Devices

| Device | Simulated Features |
|---|---|
| **Motors** (ports 1–21) | Voltage control, position/velocity/torque telemetry, brake modes (coast/brake/hold), torque-speed curve |
| **Motor Groups** | Multi-motor grouping, same API as hardware |
| **IMU** | Heading, rotation, pitch, roll, yaw |
| **Rotation Sensors** | Position & velocity (supports dead-wheel tracking) |
| **Distance Sensors** | Simulated distance readings with beam visualization |
| **Controller** | Analog sticks & digital buttons (keyboard-mapped) |
| **ADI / Pneumatics** | Stub support |

---

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| **C++ compiler** | GCC / MinGW with C++20 | Tested with MSYS2 UCRT64 on Windows |
| **CMake** | ≥ 3.16 | |
| **Ninja** | any | Or another CMake generator |
| **Python** | ≥ 3.12 | |
| **uv** | any | Python package manager ([install](https://astral.sh/uv/install)) — or use pip/venv |

---

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/<your-org>/pros-hybrid.git
cd pros-hybrid
```

### 2. Build the C++ executable

```bash
cd pros-sim
mkdir build && cd build
cmake .. -G Ninja
ninja
```

This produces `sim_client.exe` (Windows) or `sim_client` (Linux/macOS).

### 3. Start the Python simulator

```bash
cd vex-simulator

# Option A — using uv (recommended)
uv sync
uv run python -m vex_simulator

# Option B — using pip
python -m venv .venv
.venv\Scripts\activate        # Windows
# source .venv/bin/activate   # Linux / macOS
pip install -e .
python -m vex_simulator
```

The simulator opens a Pygame window showing the field and prints:

```
Simulator running on 127.0.0.1:9090
Waiting for client connections...
```

### 4. Run your robot code

In a **second terminal**:

```bash
cd pros-sim/build
./sim_client          # Linux / macOS
.\sim_client.exe      # Windows
```

Your `initialize()` and `autonomous()` functions execute against the simulated field — watch the robot move in real time!

---

## Adapting the Project for Your Team

The entire point of this project is that **you bring your own robot code**. Here's the step-by-step:

### Step 1 — Configure your robot

Edit `vex-simulator/robot_config.toml` to match your physical build:

```toml
[drivetrain]
wheel_diameter_in   = 3.25          # 2.75, 3.25, or 4.0
track_width_in      = 11.67         # center-to-center, inches
motor_max_rpm       = 450.0         # green=200, blue=600, red=100
left_motor_ports    = [-1, -2, -4]  # negative = reversed
right_motor_ports   = [11, 12, 13]

[chassis]
robot_size_in = 18.0

[physics]
robot_mass_kg          = 10
robot_moi_kg_m2        = 0.17
rolling_friction_coeff = 0.1

[start_pose]
x_in        = 81.0
y_in        = 27.0
heading_deg = 0.0

[sensors]
imu_port = 16

[[sensors.rotation]]
port              = -18
offset_forward    = -5.0
offset_right      = 0
orientation       = "forward"
wheel_diameter_in = 2
```

### Step 2 — Drop in your source files

Copy your existing `.cpp` and `.h` files into `pros-sim/src/` and `pros-sim/include/`. The key constraint is:

- Your code must use the standard PROS API (`pros/motors.hpp`, `pros/imu.hpp`, etc.).
- Remove any hardware-only includes that don't have stubs (rare — most of the PROS API is covered).

Your `main.cpp` should define the standard PROS entry points:

```cpp
void initialize()             { /* ... */ }
void autonomous()             { /* ... */ }
void opcontrol()              { /* ... */ }
void disabled()               { /* ... */ }
void competition_initialize() { /* ... */ }
```

### Step 3 — Update CMakeLists.txt (if needed)

The default `CMakeLists.txt` automatically globs `src/*.cpp`. If you add subdirectories, add another `file(GLOB ...)` line:

```cmake
file(GLOB USER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subsystems/*.cpp   # ← add your folders
)
```

### Step 4 — Build & run

```bash
# Terminal 1 — simulator
cd vex-simulator && uv run python -m vex_simulator

# Terminal 2 — robot code
cd pros-sim/build && cmake .. -G Ninja && ninja && ./sim_client
```

---

## Path Editor

Design autonomous paths visually:

```bash
cd vex-simulator
uv run python -m vex_simulator editor
```

- **Click** on the field to place waypoints.
- **Drag** waypoints to reposition them.
- **Drag the heading handle** (yellow dot) to set the robot's desired heading at that point.
- Press **S** to export the path as a C++ `CatmullRomPath` definition you can paste into your source.

Paths are automatically loaded and overlaid on the simulator visualization.

---

## Project Structure

```
pros-hybrid/
├── README.md                  ← you are here
├── pros-hardware/             ← original PROS project (deploys to V5 brain)
│   ├── src/main.cpp
│   ├── include/
│   └── Makefile
│
├── pros-sim/                  ← desktop-compiled PROS stubs + your code
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── pros/              ← unmodified PROS headers
│   │   ├── sim/sim_client.h   ← TCP client singleton
│   │   └── *.h                ← your headers
│   └── src/
│       ├── pros/              ← stub .cpp files (forward to SimClient)
│       ├── sim/               ← SimClient + sim entry point
│       └── *.cpp              ← your source files
│
└── vex-simulator/             ← Python physics simulator & visualizer
    ├── pyproject.toml
    ├── robot_config.toml      ← your robot's configuration
    └── vex_simulator/
        ├── __main__.py        ← entry point
        ├── server.py          ← TCP server (async)
        ├── devices.py         ← motor / sensor models
        ├── robot_state.py     ← physics integration
        ├── protocol.py        ← JSON message definitions
        ├── visualizer.py      ← Pygame field renderer
        ├── path_editor.py     ← visual path designer
        ├── path_loader.py     ← reads paths from C++ source
        └── config.py          ← reads robot_config.toml
```

---

## How It Works (Detail)

1. The **Python simulator** starts a TCP server on `127.0.0.1:9090` and begins a physics loop at 50 Hz.
2. The **C++ executable** connects, receives a handshake, then calls your `initialize()` → `autonomous()` → `opcontrol()` entry points.
3. When your code calls a PROS function like `motor_move(1, 127)`, the stub sends a JSON command:
   ```json
   {"type": "motor_move", "port": 1, "voltage": 127}
   ```
4. The simulator applies the voltage to its motor model (DC torque-speed curve with dead-zone and brake modes), integrates physics (Euler), and broadcasts the full robot state back to all clients:
   ```json
   {"type": "state_update", "motors": {...}, "imu": {...}, "rotation": {...}, ...}
   ```
5. The C++ `SimClient` caches the latest state so calls like `motor_get_actual_velocity(1)` or `imu_get_heading(16)` return instantly from the cache.
6. The **Pygame visualizer** renders the robot on a 144 × 144 in. field image with a live telemetry side-panel, distance-sensor beams, an odometry trail, and any loaded spline paths.

---

## Tips

- **Iterate fast** — change your autonomous routine, rebuild with `ninja`, and re-run. No need to restart the simulator.
- **Tune PID on desktop** — adjust gains in your code, rebuild, and watch the robot's response in the visualizer.
- **Keep one codebase** — `pros-hardware/` deploys to the real V5 brain with `pros mu`. `pros-sim/` shares the same headers and logic files, just compiled differently.
- **Swap field images** — replace `vex-simulator/assets/Skills_field.png` with the current season's field to match your competition layout.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Failed to connect to simulator` | Make sure the Python simulator is running first. |
| Build errors about missing headers | Ensure `pros-sim/include/pros/` contains all standard PROS headers. |
| Robot doesn't move | Check that `robot_config.toml` motor ports match the ports used in your code. Negative port = reversed. |
| Simulator window doesn't open | Install pygame: `pip install pygame>=2.0.0` |
| Linker errors (multiple definitions) | The CMake config uses `--allow-multiple-definition` for MinGW. If using a different toolchain, add the equivalent flag. |

---

## Contributing

1. Fork the repo.
2. Create a feature branch.
3. Submit a pull request.

Bug reports and feature requests are welcome via GitHub Issues.

---

## License

MIT — use it, fork it, share it with your alliance partners.
