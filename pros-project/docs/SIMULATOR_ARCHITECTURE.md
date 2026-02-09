# PROS Simulator Client Library — Architecture Design

## 1. Goal

Create a **drop-in replacement library** (called **`pros-sim`** throughout this document) that exposes the exact same `pros::` namespace, classes, functions, enums, and macros as the real PROS kernel, but instead of talking to VEX V5 hardware, it communicates with an external robot simulator over a local transport (TCP socket, shared memory, or similar). 

A user's robot code (including libraries like LemLib built on PROS) can then be compiled into **two separate projects** from a single shared source tree:

| Project | Compiles Against | Target | Output |
|---|---|---|---|
| **Robot** | Real PROS kernel (`libpros.a`, ARM cross-compiler) | VEX V5 Brain | `monolith.bin` / `hot.package.bin` |
| **Simulator** | `pros-sim` (MSVC / MinGW / Clang, native x86-64) | Windows `.exe` | Connects to your simulator process |

---

## 2. Key Constraints

1. **Binary API compatibility is NOT needed** — we only need **source-level (header) compatibility**. The user recompiles their code against our headers.  
2. **Same header file paths** — `#include "pros/motors.hpp"` must resolve to our replacement headers. The include directory structure must mirror the original exactly.
3. **Same namespaces & class hierarchies** — `pros::v5::Motor`, `pros::v5::MotorGroup`, `pros::rtos::Task`, `pros::lcd::*`, etc.
4. **Same enums, typedefs, and macros** — `ANALOG_LEFT_Y`, `E_CONTROLLER_MASTER`, `LCD_BTN_LEFT`, `TASK_PRIORITY_DEFAULT`, etc.
5. **C API surface too** — many PROS C++ classes are thin wrappers around C functions (e.g., `motor_move()`). Both layers must exist.
6. **RTOS primitives** — `pros::Task`, `pros::Mutex`, `pros::delay()` must work. On Windows, these map to `std::thread`, `std::mutex`, `std::this_thread::sleep_for`, etc.
7. **LemLib compatibility** — LemLib depends on Motors, MotorGroup, AbstractMotor, IMU, Rotation, Controller, RTOS (Task/Mutex/delay). These are the highest priority.

---

## 3. High-Level Architecture

```
┌─────────────────────────────────────────────────┐
│              User Robot Code (shared)            │
│         src/main.cpp, src/autonomous.cpp, ...    │
│         + LemLib / other PROS libraries          │
├─────────────────────────────────────────────────┤
│         #include "pros/motors.hpp" etc.          │
├────────────────┬────────────────────────────────┤
│  Real PROS     │    pros-sim library             │
│  (ARM, V5)     │    (x86-64, Windows)            │
│                │                                 │
│  libpros.a     │  ┌───────────────────────────┐  │
│  libc.a        │  │  API Layer                │  │
│  libm.a        │  │  Same headers as PROS     │  │
│                │  │  Same classes & functions  │  │
│                │  ├───────────────────────────┤  │
│                │  │  Simulation Backend       │  │
│                │  │  - Device registry        │  │
│                │  │  - State tracking         │  │
│                │  │  - RTOS emulation         │  │
│                │  ├───────────────────────────┤  │
│                │  │  Transport Layer          │  │
│                │  │  - TCP / WebSocket /      │  │
│                │  │    shared memory to sim   │  │
│                │  └───────────────────────────┘  │
├────────────────┼────────────────────────────────┤
│  VEX V5 Brain  │  Robot Simulator Process        │
└────────────────┴────────────────────────────────┘
```

---

## 4. Project / Directory Structure

```
pros-sim/
├── CMakeLists.txt                  # CMake build (targets Windows x86-64)
├── README.md
├── include/                        # PUBLIC headers — mirror PROS layout exactly
│   ├── api.h                       # Master include (same as PROS)
│   ├── main.h                      # Provided for convenience / can be overridden
│   ├── pros/
│   │   ├── abstract_motor.hpp      # pros::v5::AbstractMotor (pure virtual interface)
│   │   ├── adi.h                   # C ADI functions
│   │   ├── adi.hpp                 # pros::v5::adi::* classes
│   │   ├── ai_vision.h
│   │   ├── ai_vision.hpp
│   │   ├── apix.h                  # Extended API
│   │   ├── colors.h
│   │   ├── colors.hpp
│   │   ├── device.h
│   │   ├── device.hpp              # pros::v5::Device base class
│   │   ├── distance.h
│   │   ├── distance.hpp
│   │   ├── error.h
│   │   ├── ext_adi.h
│   │   ├── gps.h
│   │   ├── gps.hpp
│   │   ├── imu.h
│   │   ├── imu.hpp
│   │   ├── link.h
│   │   ├── link.hpp
│   │   ├── llemu.h
│   │   ├── llemu.hpp
│   │   ├── misc.h                  # C controller / competition functions
│   │   ├── misc.hpp                # pros::Controller
│   │   ├── motor_group.hpp
│   │   ├── motors.h
│   │   ├── motors.hpp
│   │   ├── optical.h
│   │   ├── optical.hpp
│   │   ├── rotation.h
│   │   ├── rotation.hpp
│   │   ├── rtos.h                  # C RTOS functions
│   │   ├── rtos.hpp                # pros::Task, pros::Mutex, pros::delay
│   │   ├── screen.h
│   │   ├── screen.hpp
│   │   ├── serial.h
│   │   ├── serial.hpp
│   │   ├── version.h
│   │   ├── vision.h
│   │   └── vision.hpp
│   └── liblvgl/                    # Stub / minimal LVGL headers (optional)
│       ├── llemu.h
│       └── llemu.hpp
├── src/
│   ├── api/                        # Implementation of every PROS API function
│   │   ├── motors.cpp              # Motor, MotorGroup, AbstractMotor impls
│   │   ├── controller.cpp          # Controller impl
│   │   ├── imu.cpp
│   │   ├── rotation.cpp
│   │   ├── gps.cpp
│   │   ├── distance.cpp
│   │   ├── optical.cpp
│   │   ├── adi.cpp
│   │   ├── vision.cpp
│   │   ├── ai_vision.cpp
│   │   ├── screen.cpp
│   │   ├── llemu.cpp
│   │   ├── device.cpp
│   │   ├── link.cpp
│   │   └── serial.cpp
│   ├── rtos/                       # RTOS emulation layer
│   │   ├── task.cpp                # pros::Task → std::thread wrapper
│   │   ├── mutex.cpp               # pros::Mutex → std::mutex wrapper
│   │   └── rtos_functions.cpp      # delay(), millis(), micros()
│   ├── sim/                        # Simulator communication backend
│   │   ├── sim_transport.h         # Abstract transport interface
│   │   ├── sim_transport.cpp
│   │   ├── sim_tcp_transport.cpp   # TCP implementation
│   │   ├── sim_device_registry.h   # Maps port → simulated device state
│   │   ├── sim_device_registry.cpp
│   │   ├── sim_motor_model.h       # Motor physics / state tracking
│   │   ├── sim_motor_model.cpp
│   │   └── sim_protocol.h          # Message format definitions
│   └── entry/
│       └── main_entry.cpp          # Windows main() that calls initialize()/opcontrol()
└── examples/
    └── basic/                      # Example sim project using shared robot code
        ├── CMakeLists.txt
        └── sim_config.json         # Port mappings, robot geometry, etc.
```

---

## 5. Layer-by-Layer Design

### 5.1. API Layer (Headers)

**Copy the original PROS headers verbatim and modify only the implementation details.** This is the most reliable path to compatibility:

- Start from the real PROS `include/pros/*.h` and `include/pros/*.hpp` files.
- Keep every class declaration, enum, typedef, macro, and function signature **identical**.  
- Remove any `#include` references to ARM-specific or FreeRTOS-specific internals.
- Replace internal type references (e.g., `task_t` which is a FreeRTOS handle) with platform-appropriate types wrapped in the same typedef.

**Critical types to remap:**

| PROS Type | Original (ARM/FreeRTOS) | pros-sim Replacement |
|---|---|---|
| `task_t` | `void*` (FreeRTOS task handle) | `void*` (wraps `std::thread*` internally) |
| `mutex_t` | `void*` (FreeRTOS semaphore) | `void*` (wraps `std::mutex*` internally) |
| `task_fn_t` | `void (*)(void*)` | Same — no change needed |
| `sem_t` | `void*` (FreeRTOS semaphore) | `void*` (wraps `std::counting_semaphore*`) |

### 5.2. Simulation Backend

Each PROS device class (Motor, IMU, Rotation, etc.) delegates to a **SimDeviceRegistry** — a singleton that manages simulated device state per port:

```cpp
// Internal to pros-sim, not exposed in public headers
namespace pros_sim {

class SimDeviceRegistry {
public:
    static SimDeviceRegistry& instance();

    // Motor state
    void set_motor_voltage(uint8_t port, int32_t voltage);
    double get_motor_position(uint8_t port);
    double get_motor_velocity(uint8_t port);
    double get_motor_torque(uint8_t port);
    // ... all motor telemetry

    // IMU state
    double get_imu_heading(uint8_t port);
    double get_imu_rotation(uint8_t port);
    pros_imu_raw_s get_imu_accel(uint8_t port);
    // ... etc.

    // Controller state
    int32_t get_controller_analog(controller_id_e_t id, controller_analog_e_t channel);
    int32_t get_controller_digital(controller_id_e_t id, controller_digital_e_t button);

    // Transport connection
    void connect(const std::string& host, uint16_t port);
    void disconnect();

private:
    std::unique_ptr<SimTransport> transport_;
    std::unordered_map<uint8_t, DeviceState> devices_;
    std::mutex registry_mutex_;
};

} // namespace pros_sim
```

**Example: How `pros::Motor::move()` works in pros-sim:**

```cpp
// src/api/motors.cpp
namespace pros {
inline namespace v5 {

std::int32_t Motor::move(std::int32_t voltage) {
    // Clamp to [-127, 127] like real PROS
    voltage = std::clamp(voltage, -127, 127);
    // Send to simulator
    pros_sim::SimDeviceRegistry::instance().set_motor_voltage(_port, voltage);
    return 1; // success
}

double Motor::get_position() {
    return pros_sim::SimDeviceRegistry::instance().get_motor_position(_port);
}

} // namespace v5
} // namespace pros
```

### 5.3. Transport Layer

The transport is an abstract interface so you can swap implementations:

```cpp
namespace pros_sim {

// Message sent to/from the simulator
struct SimMessage {
    enum class Type : uint8_t {
        // Commands (client → simulator)
        SET_MOTOR_VOLTAGE,
        SET_MOTOR_POSITION,
        SET_MOTOR_BRAKE_MODE,
        // Queries (client → simulator, simulator → client)
        GET_MOTOR_POSITION,
        GET_MOTOR_VELOCITY,
        GET_IMU_HEADING,
        GET_CONTROLLER_ANALOG,
        GET_CONTROLLER_DIGITAL,
        // Bulk state update (simulator → client, periodic)
        STATE_UPDATE,
        // Lifecycle
        HANDSHAKE,
        DISCONNECT,
    };
    Type type;
    uint8_t port;           // device port (1-21)
    uint32_t param;         // enum/channel selector
    double value;           // numeric payload
    // For bulk updates, use a separate struct
};

class SimTransport {
public:
    virtual ~SimTransport() = default;
    virtual bool connect(const std::string& host, uint16_t port) = 0;
    virtual void disconnect() = 0;
    virtual bool send(const SimMessage& msg) = 0;
    virtual bool receive(SimMessage& msg) = 0;

    // Bulk state — simulator pushes complete robot state at fixed rate
    virtual bool receive_state_update(RobotState& state) = 0;
};

class TcpTransport : public SimTransport { /* ... Winsock2 implementation ... */ };

} // namespace pros_sim
```

**Recommended communication pattern:** Use a **hybrid push/pull model**:

- **Push (client → sim):** Motor commands are sent immediately when `move()` is called.
- **Pull (sim → client):** The simulator pushes a full `RobotState` snapshot at a fixed rate (e.g., 50Hz / every 20ms, matching PROS's typical `delay(20)` loop). This state includes all sensor readings, motor telemetry, and controller inputs. The `SimDeviceRegistry` caches this state, and API getters just read from the cache — no round-trip latency.

### 5.4. RTOS Emulation

This is critical for LemLib and any code that uses `pros::Task` or `pros::Mutex`:

```cpp
// src/rtos/task.cpp
namespace pros {
inline namespace rtos {

Task::Task(task_fn_t function, void* parameters, uint32_t prio,
           uint16_t stack_depth, const char* name) {
    // Ignore prio and stack_depth (not meaningful on desktop)
    auto thread = new std::thread([function, parameters]() {
        function(parameters);
    });
    task = static_cast<task_t>(thread);
}

// pros::delay() — maps to std::this_thread::sleep_for
void delay(uint32_t milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

uint32_t millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

} // namespace rtos
} // namespace pros
```

**Task notification** functions (`task_notify`, `task_notify_take`, etc.) can be implemented with `std::condition_variable` + atomic counters.

### 5.5. Entry Point

The real PROS kernel calls `initialize()`, then `opcontrol()` (or `autonomous()` depending on competition state). Your simulator entry point replicates this:

```cpp
// src/entry/main_entry.cpp
#include "main.h"
#include "sim/sim_device_registry.h"

// Forward declarations from user code (declared in main.h)
extern "C" {
    void initialize(void);
    void opcontrol(void);
    void autonomous(void);
    void disabled(void);
    void competition_initialize(void);
}

int main(int argc, char* argv[]) {
    // Parse config (simulator host, port, robot config)
    std::string sim_host = "127.0.0.1";
    uint16_t sim_port = 9090;
    if (argc > 1) sim_host = argv[1];
    if (argc > 2) sim_port = std::atoi(argv[2]);

    // Connect to simulator
    pros_sim::SimDeviceRegistry::instance().connect(sim_host, sim_port);

    // Start the state-receive background thread
    pros_sim::SimDeviceRegistry::instance().start_state_listener();

    // Mirror PROS boot sequence
    initialize();
    competition_initialize();
    opcontrol();  // or autonomous(), driven by simulator commands

    pros_sim::SimDeviceRegistry::instance().disconnect();
    return 0;
}
```

---

## 6. How Users Structure Their Projects

### 6.1. Shared Code Layout

```
my-robot/
├── shared/                     # All robot logic lives here
│   ├── src/
│   │   ├── main.cpp            # initialize(), opcontrol(), autonomous()
│   │   ├── drivetrain.cpp
│   │   └── auton_routines.cpp 
│   └── include/
│       ├── main.h              # Standard PROS main.h
│       ├── drivetrain.h
│       └── auton_routines.h
│
├── robot/                      # Real PROS project (for V5 hardware)
│   ├── Makefile                # Standard PROS Makefile
│   ├── common.mk
│   ├── project.pros
│   ├── include/ → ../shared/include  (symlink or copy)
│   ├── src/ → ../shared/src          (symlink or copy)
│   └── firmware/               # libpros.a, libc.a, etc.
│
└── simulator/                  # Simulator project
    ├── CMakeLists.txt
    └── (links to ../shared/src and ../shared/include)
```

### 6.2. Simulator Project CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(my-robot-sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Path to pros-sim library (installed or as submodule)
add_subdirectory(path/to/pros-sim)

# Shared robot code
file(GLOB_RECURSE ROBOT_SOURCES "${CMAKE_SOURCE_DIR}/../shared/src/*.cpp")

add_executable(robot_sim ${ROBOT_SOURCES})

target_include_directories(robot_sim PRIVATE
    "${CMAKE_SOURCE_DIR}/../shared/include"
)

# Link against pros-sim (provides all the pros:: symbols)
target_link_libraries(robot_sim PRIVATE pros-sim)
```

### 6.3. Alternative: Single Project with Preprocessor

For simpler setups, users can use a single project with a `#ifdef PROS_SIM` guard, though the two-project approach is cleaner:

```cpp
// Not recommended but possible — shows the code is truly shared
#ifdef PROS_SIM
  #include "pros-sim/api.h"
#else
  #include "api.h"
#endif
```

This should NOT be necessary if the header paths match exactly.

---

## 7. Implementation Priority / Phases

### Phase 1: Core (Minimum viable for basic opcontrol)
- [ ] **Motor** and **MotorGroup** (move, get_position, get_velocity, brake modes)
- [ ] **Controller** (analog sticks, digital buttons)
- [ ] **RTOS** (Task, Mutex, delay, millis)
- [ ] **Device** base class
- [ ] **Entry point** (main → initialize → opcontrol)
- [ ] **Transport** (basic TCP, send motor commands, receive state)
- [ ] **LCD/LLEMU** (stubs that print to console)

### Phase 2: Sensors (Required for LemLib)
- [ ] **IMU** (heading, rotation, gyro rate, accelerometer)
- [ ] **Rotation sensor** (position, velocity)
- [ ] **GPS sensor** (position, heading)
- [ ] **Distance sensor**
- [ ] **Optical sensor**

### Phase 3: Extended
- [ ] **ADI** (digital/analog I/O, encoders, ultrasonic, gyro)
- [ ] **Vision sensor**
- [ ] **AI Vision sensor**
- [ ] **Link** (V5-to-V5 radio)
- [ ] **Serial**
- [ ] **Screen** (drawing primitives)
- [ ] **Competition state management** (auto/opcontrol/disabled switching)

### Phase 4: Polish
- [ ] Full RTOS (task notifications, semaphores, queues from `apix.h`)
- [ ] Error handling / errno emulation
- [ ] Hot-reload support (reconnect without restarting sim)
- [ ] Configuration file for port mappings and robot geometry

---

## 8. Detailed API Compatibility Checklist

### 8.1. Class Hierarchy (must match exactly)

```
pros::v5::Device                    ← base class, stores _port
├── pros::v5::Motor                 ← also inherits AbstractMotor
├── pros::v5::Imu
├── pros::v5::Rotation
├── pros::v5::Distance
├── pros::v5::Optical
├── pros::v5::Gps
├── pros::v5::Vision
├── pros::v5::AIVision
├── pros::v5::Link
└── pros::v5::Serial

pros::v5::AbstractMotor             ← pure virtual interface
├── pros::v5::Motor                 ← concrete, also extends Device
└── pros::v5::MotorGroup            ← concrete, does NOT extend Device

pros::Controller                    ← standalone (not a Device)

pros::rtos::Task
pros::rtos::Mutex
pros::rtos::RecursiveMutex

pros::lcd::*                        ← namespace with free functions
```

### 8.2. Namespace Structure

```cpp
namespace pros {
    // Free functions
    void delay(uint32_t ms);
    uint32_t millis();
    uint32_t micros();
    double battery::get_capacity();
    // etc.

    inline namespace v5 {
        class Device { ... };
        class Motor : public AbstractMotor, public Device { ... };
        class MotorGroup : public AbstractMotor { ... };
        class Imu : public Device { ... };
        // etc.
    }

    inline namespace rtos {
        class Task { ... };
        class Mutex { ... };
    }

    namespace lcd {
        bool initialize();
        void set_text(int line, std::string text);
        void print(int line, const char* fmt, ...);
        // etc.
    }
}
```

The use of `inline namespace` is important — it allows `pros::Motor` and `pros::v5::Motor` to both work.

### 8.3. C Functions (must be `extern "C"`)

Every C++ class method in PROS wraps an underlying C function. These must also exist:

```c
// motors.h
int32_t motor_move(int8_t port, int32_t voltage);
double motor_get_position(int8_t port);
double motor_get_actual_velocity(int8_t port);
// ... ~40 motor C functions

// misc.h  
int32_t controller_get_analog(controller_id_e_t id, controller_analog_e_t channel);
int32_t controller_get_digital(controller_id_e_t id, controller_digital_e_t button);

// rtos.h
void delay(uint32_t ms);
uint32_t millis(void);
task_t task_create(task_fn_t function, void* parameters, ...);
```

### 8.4. Macros & Constants

```c
// These must be defined exactly as PROS defines them
#define PROS_ERR        INT32_MAX
#define PROS_ERR_F      INFINITY
#define ANALOG_LEFT_X   0
#define ANALOG_LEFT_Y   1
#define ANALOG_RIGHT_X  2
#define ANALOG_RIGHT_Y  3
#define LCD_BTN_LEFT    4
#define LCD_BTN_CENTER  2
#define LCD_BTN_RIGHT   1
#define TASK_PRIORITY_DEFAULT   8
#define TASK_STACK_DEPTH_DEFAULT 0x2000
// etc.
```

---

## 9. Build System

### 9.1. pros-sim Library Build

Use CMake targeting native Windows:

```cmake
cmake_minimum_required(VERSION 3.20)
project(pros-sim VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

file(GLOB_RECURSE PROS_SIM_SOURCES "src/*.cpp")

add_library(pros-sim STATIC ${PROS_SIM_SOURCES})

target_include_directories(pros-sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Winsock for TCP transport
if(WIN32)
    target_link_libraries(pros-sim PRIVATE ws2_32)
endif()

# Optional: install target
install(TARGETS pros-sim ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
```

### 9.2. Why Not the PROS Makefile?

The PROS Makefile uses `arm-none-eabi-gcc` and links against FreeRTOS. The simulator project must use a native compiler (MSVC, MinGW-w64, or Clang). CMake is the natural choice for portable native C++ builds on Windows.

---

## 10. Handling LemLib Specifically

LemLib's key dependencies on PROS:

| LemLib Uses | PROS Class/Function | Sim Must Provide |
|---|---|---|
| Drivetrain motors | `Motor`, `MotorGroup`, `AbstractMotor` | Full motor API |
| Tracking sensors | `Imu`, `Rotation` | Heading, position, velocity |
| PID control loop | `pros::delay()`, `pros::millis()` | Timing |
| Background tasks | `pros::Task` | Thread wrapper |  
| Thread safety | `pros::Mutex` | Mutex wrapper |
| Controller input | `pros::Controller` | Analog/digital reads |

LemLib does NOT typically use: Vision, AI Vision, Link, Serial, ADI, Screen drawing, GPS. So Phase 1 + Phase 2 (IMU + Rotation) should be sufficient.

---

## 11. Simulator Protocol Sketch

### State Update (Sim → Client, 50Hz)

```json
{
  "timestamp_ms": 12345,
  "motors": {
    "1": { "position_deg": 180.5, "velocity_rpm": 195.2, "torque_nm": 0.8, "temp_c": 35.0, "current_ma": 800 },
    "2": { "position_deg": -90.0, "velocity_rpm": -198.1, "torque_nm": 0.7, "temp_c": 34.5, "current_ma": 750 }
  },
  "imu": {
    "5": { "heading": 45.0, "rotation": -315.0, "pitch": 0.1, "roll": 0.2, "yaw": 45.0 }
  },
  "rotation": {
    "8": { "position_cdeg": 18050, "velocity_cdps": 1200 }
  },
  "controller": {
    "master": { "left_x": 0, "left_y": 127, "right_x": -30, "right_y": 0, "buttons": 0 },
    "partner": { "left_x": 0, "left_y": 0, "right_x": 0, "right_y": 0, "buttons": 0 }
  },
  "competition": { "state": "opcontrol", "connected": true }
}
```

### Commands (Client → Sim)

```json
{ "type": "motor_move", "port": 1, "voltage": 127 }
{ "type": "motor_move_velocity", "port": 1, "velocity": 200 }
{ "type": "motor_set_brake_mode", "port": 1, "mode": "hold" }
{ "type": "imu_reset", "port": 5 }
{ "type": "lcd_set_text", "line": 1, "text": "Hello!" }
```

Using JSON makes debugging easy. For performance-critical scenarios, switch to a binary protocol later.

---

## 12. Risks and Mitigations

| Risk | Mitigation |
|---|---|
| PROS updates break compatibility | Pin to a specific PROS kernel version (e.g., 4.2.1). Diff new headers against yours when upgrading. |
| Timing differences (desktop vs V5) | Use a virtual clock that the simulator controls. All `millis()`/`delay()` calls go through this clock. Optionally run in lockstep with sim. |
| FreeRTOS-specific behavior (priorities, stack size) | Document that task priorities are best-effort. Use `std::thread` with no priority manipulation. Most robot code doesn't depend on exact scheduling. |
| Missing C runtime stubs | The real PROS links `libc.a`/`libm.a` for ARM. On Windows, the native C runtime handles this. Just don't include the ARM libraries. |
| LVGL dependency | If code uses LVGL directly (not just `pros::lcd`), provide minimal stubs or a headless LVGL port. Most competitive code only uses `pros::lcd`. |

---

## 13. Getting Started — First Steps

1. **Create the `pros-sim` repository** with the directory structure from Section 4.
2. **Copy PROS headers** from your PROS project's `include/pros/` into `pros-sim/include/pros/`. Strip ARM/FreeRTOS internals.
3. **Implement stubs** for every function — start with returning `PROS_ERR` or `0` for everything.
4. **Implement RTOS** (Task, Mutex, delay, millis) — these are needed first since most other code depends on them.
5. **Implement Motor::move()** and Controller analog reads with a simple TCP transport.
6. **Build a minimal simulator** that accepts motor voltage commands and sends back position/velocity state.
7. **Compile the default PROS template `main.cpp`** against pros-sim and verify it links and runs.
8. **Iterate:** add more device implementations, refine the protocol, test with LemLib.
