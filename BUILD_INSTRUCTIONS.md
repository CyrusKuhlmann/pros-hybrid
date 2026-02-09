# Complete Build and Test Instructions

## Project Structure

```
PROS_TEST/
├── PROS_TEST/              # Real PROS project (for VEX V5 hardware)
│   ├── Makefile
│   ├── src/                # Update to use shared code
│   └── include/            # Update to use shared code
└── shared/                 # Shared robot behavior code
│   ├── src/main.cpp        # Robot logic
│   └── include/main.h
├── pros-sim/               # Simulator client library (C++)
│   ├── CMakeLists.txt
│   ├── include/pros/       # PROS API headers
│   └── src/                # Simulator backend implementation
├── robot-simulator/        # Simulator executable project
│   ├── CMakeLists.txt
│   └── main_entry.cpp
└── vex-simulator/          # Python simulator server
    ├── pyproject.toml
    └── vex_simulator/

```

## Setup Steps

### 1. Python Simulator

```powershell
cd vex-simulator
# Install uv if needed: https://docs.astral.sh/uv/getting-started/installation/
uv sync
```

### 2. C++ Simulator Client

Requires CMake and a C++ compiler (MSVC, MinGW, or Clang).

```powershell
cd robot-simulator
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### 3. Real PROS Project

Update the real PROS project to use shared code (see SHARED_CODE_SETUP.md), then:

```powershell
cd PROS_TEST
pros make
```

## Running the Simulation

### Terminal 1: Start Python Simulator

```powershell
cd vex-simulator
uv run python -m vex_simulator
```

You should see:
```
Simulator running on 127.0.0.1:9090
Waiting for client connections...
```

### Terminal 2: Run Robot Code

```powershell
cd robot-simulator\build
.\Release\robot-sim.exe
```

You should see:
```
Connecting to simulator at 127.0.0.1:9090...
Connected to simulator at 127.0.0.1:9090
Connected! Starting robot code...
[LCD] Initialized
[LCD Line 1] Robot Initialized
[LCD Line 0] Starting in 1s...
[LCD Line 0] Driving straight...
Motor 1: voltage=100
Motor 2: voltage=-100
Motor 3: voltage=100
...
```

The robot will drive straight for 5 seconds, then enter controller mode and display IMU heading.

## Uploading to Real Robot

After testing in simulator:

```powershell
cd PROS_TEST
pros upload
```

Then run opcontrol on the V5 brain.

## Troubleshooting

### Simulator won't connect
- Ensure Python simulator is running first
- Check firewall isn't blocking port 9090
- Try running both on same machine

### CMake build fails
- Ensure CMake 3.20+ installed: `cmake --version`
- On Windows, ensure you have Visual Studio or MinGW installed
- Try: `cmake .. -G "Visual Studio 17 2022"` or `cmake .. -G "MinGW Makefiles"`

### PROS build fails
- Ensure PROS CLI installed: `pros --version`
- Run: `pros conduct apply kernel@4.2.1`
