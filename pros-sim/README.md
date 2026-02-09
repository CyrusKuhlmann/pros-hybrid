# PROS Simulator Client Library

Drop-in replacement for PROS that communicates with a robot simulator instead of VEX V5 hardware.

## Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Usage

Include in your simulator project:

```cmake
add_subdirectory(path/to/pros-sim)
target_link_libraries(your_robot_sim PRIVATE pros-sim)
```

## Architecture

- **API Layer**: Identical headers to PROS (`pros/motors.hpp`, `pros/rtos.hpp`, etc.)
- **Simulation Backend**: TCP client that connects to Python simulator
- **RTOS Emulation**: Maps PROS tasks/mutexes to `std::thread`/`std::mutex`

## Supported API

- Motors (move, position, velocity, brake modes)
- IMU (heading, rotation)
- Rotation sensors
- Distance sensors
- Controller (analog/digital inputs)
- RTOS (Task, Mutex, delay, millis)
- LCD/LLEMU (console output stubs)
