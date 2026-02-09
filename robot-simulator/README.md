# Robot Simulator Project

Simulator version of the robot code that links against `pros-sim` instead of real PROS.

## Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Run

1. Start the Python simulator:
```bash
cd ../vex-simulator
uv run python -m vex_simulator
```

2. Run the simulator client:
```bash
./robot-sim.exe   # or robot-sim on Linux
```

The client will automatically connect to `127.0.0.1:9090`.

## Shared Code

Robot behavior code is in `../shared/src/main.cpp` and is shared between:
- Real PROS project (`PROS_TEST/`)
- Simulator project (this directory)
