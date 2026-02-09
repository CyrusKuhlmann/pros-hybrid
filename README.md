# PROS Simulator - Dual Compilation Architecture

Test your VEX V5 robot code in a simulator before uploading to hardware. Write your robot logic once, compile it twice: once for the simulator (Windows .exe) and once for the real robot (VEX V5 ARM binary).

## Quick Start

### 1. Start the Simulator Server

```powershell
cd vex-simulator
uv sync  # First time only
uv run python -m vex_simulator
```

### 2. Build and Run Robot Code

```powershell
cd robot-simulator
mkdir build && cd build
cmake ..
cmake --build . --config Release
.\robot-sim.exe
```

You should see motor commands and LCD output in both terminals!

### 3. Upload to Real Robot

```powershell
# First, link shared code (see SHARED_CODE_SETUP.md)
cd PROS_TEST
pros make
pros upload
```

## Project Structure

```
├── vex-simulator/          Python simulator (TCP server, physics engine)
├── pros-sim/               C++ library (PROS API replacement) 
├── robot-simulator/        Simulator executable (links pros-sim + shared code)
├── shared/                 Your robot logic (used by both sim and real robot)
└── PROS_TEST/             Real PROS project (for VEX V5 hardware)
```

## What's Implemented

- **Motors**: Motor, MotorGroup with voltage control and telemetry
- **Sensors**: IMU (heading/rotation), Rotation, Distance
- **Controller**: Analog sticks and digital buttons
- **RTOS**: Task (threads), Mutex, delay/millis
- **LCD**: Console output for debugging

See [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) for complete details.

## Documentation

- **[BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)** - Detailed build and run instructions
- **[SHARED_CODE_SETUP.md](SHARED_CODE_SETUP.md)** - How to configure real PROS project
- **[docs/SIMULATOR_ARCHITECTURE.md](docs/SIMULATOR_ARCHITECTURE.md)** - Full architecture design
- **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)** - What was built and how

## Requirements

- **Python 3.12+** with `uv` package manager
- **CMake 3.20+** 
- **C++ Compiler** (MSVC, MinGW, or Clang)
- **PROS CLI** (for real robot uploads)

## Benefits

✅ **Faster iteration** - No compilation for ARM, no upload time  
✅ **Debugging** - Use standard C++ debuggers instead of V5 brain  
✅ **CI/CD** - Automated testing without hardware  
✅ **Team development** - Test without monopolizing the robot  
✅ **LemLib compatible** - Works with existing PROS libraries  

## Example Robot Code

```cpp
void opcontrol() {
    pros::MotorGroup left({1, -2, 3});
    pros::MotorGroup right({-4, 5, -6});
    pros::Imu imu(7);
    
    while (true) {
        double heading = imu.get_heading();
        pros::lcd::print(0, "Heading: %.2f", heading);
        
        left.move(100);
        right.move(100);
        pros::delay(20);
    }
}
```

This exact code compiles for both simulator and real robot!

## Next Steps

1. Follow [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) to build everything
2. See [SHARED_CODE_SETUP.md](SHARED_CODE_SETUP.md) to configure your PROS project
3. Read [docs/SIMULATOR_ARCHITECTURE.md](docs/SIMULATOR_ARCHITECTURE.md) for implementation phases
4. Start writing robot code in `shared/src/main.cpp`!

## License

This implementation follows the same license as PROS (Mozilla Public License 2.0).
