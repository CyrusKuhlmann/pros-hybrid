# PROS V5 Complete API Surface Reference

> **Purpose**: Comprehensive reference of every type, function, class, enum, constant, and struct in the PROS V5 API, for use in building a simulator that reimplements this entire surface.
>
> **Source**: PROS Kernel headers, Copyright 2017-2024 Purdue University ACM SIGBots (MPL 2.0)
>
> **Architecture**: Dual C/C++ API. C functions live in `extern "C"` / `pros::c` namespace. C++ classes live in `pros` namespace with inline namespaces `pros::v5` (devices) and `pros::rtos` (RTOS). All headers included via `api.h` / `main.h`.

---

## Table of Contents

1. [Entry Points & Lifecycle](#1-entry-points--lifecycle)
2. [Error Handling](#2-error-handling)
3. [Device Base](#3-device-base)
4. [Motors](#4-motors)
5. [Motor Groups](#5-motor-groups)
6. [Controller & Misc](#6-controller--misc)
7. [ADI (3-Wire)](#7-adi-3-wire)
8. [ADI Expander (ext_adi)](#8-adi-expander-ext_adi)
9. [IMU (Inertial)](#9-imu-inertial)
10. [Rotation Sensor](#10-rotation-sensor)
11. [Distance Sensor](#11-distance-sensor)
12. [Optical Sensor](#12-optical-sensor)
13. [GPS Sensor](#13-gps-sensor)
14. [Vision Sensor](#14-vision-sensor)
15. [AI Vision Sensor](#15-ai-vision-sensor)
16. [Screen (Brain Display)](#16-screen-brain-display)
17. [LLEMU (Legacy LCD Emulator)](#17-llemu-legacy-lcd-emulator)
18. [VEXlink (Radio)](#18-vexlink-radio)
19. [Serial (Generic UART)](#19-serial-generic-uart)
20. [RTOS](#20-rtos)
21. [Extended API (apix)](#21-extended-api-apix)
22. [Colors](#22-colors)

---

## 1. Entry Points & Lifecycle

Defined in `main.h`. User implements these five `extern "C"` functions:

```c
void initialize(void);            // Runs first on program start
void competition_initialize(void); // Runs when connected to field controller
void autonomous(void);            // Autonomous period
void opcontrol(void);             // Operator control period
void disabled(void);              // Disabled period
```

**Preprocessor Macros** (in `main.h`):
- `PROS_USE_SIMPLE_NAMES` — enables short enum names (e.g. `CONTROLLER_MASTER` instead of `E_CONTROLLER_MASTER`)
- `PROS_USE_LITERALS` — enables C++ user-defined literals in `pros::literals`

---

## 2. Error Handling

**Header**: `pros/error.h`

| Constant | Type | Value | Usage |
|---|---|---|---|
| `PROS_ERR` | `int32_t` | `INT32_MAX` (2147483647) | Error return for int32 functions |
| `PROS_ERR_F` | `double` | `INFINITY` | Error return for double functions |
| `PROS_ERR_BYTE` | `int8_t` | `INT8_MAX` | Error return for byte functions |
| `PROS_ERR_2_BYTE` | `int16_t` | `INT16_MAX` | Error return for int16 functions |
| `PROS_SUCCESS` | `int32_t` | `1` | Success return value |

**errno values used throughout**:
- `ENXIO` — port out of range
- `ENODEV` — port cannot be configured as requested device
- `EACCES` — another resource accessing port
- `EOVERFLOW` — value overflow  
- `EDOM` — domain error
- `EINVAL` — invalid argument
- `EAGAIN` — resource temporarily unavailable
- `EADDRINUSE` — port already configured differently

---

## 3. Device Base

**Headers**: `pros/device.h`, `pros/device.hpp`

### C Enum
```c
typedef enum v5_device_e {
    E_DEVICE_NONE       = 0,
    E_DEVICE_MOTOR      = 2,
    E_DEVICE_ROTATION   = 4,
    E_DEVICE_IMU        = 6,
    E_DEVICE_DISTANCE   = 7,
    E_DEVICE_RADIO      = 8,
    E_DEVICE_VISION     = 11,
    E_DEVICE_ADI        = 12,
    E_DEVICE_OPTICAL    = 16,
    E_DEVICE_GPS        = 20,
    E_DEVICE_AIVISION   = 29,
    E_DEVICE_SERIAL     = 129,
    E_DEVICE_UNDEFINED  = 255
} v5_device_e_t;
```

### C++ `class Device`
```cpp
namespace pros {
inline namespace v5 {
class Device {
public:
    Device(uint8_t port);
    
    std::int8_t get_port() const;
    bool is_installed() const;
    v5_device_e_t get_plugged_type() const;
    
    static std::vector<Device> get_all_devices(v5_device_e_t device_type = E_DEVICE_NONE);

protected:
    std::int8_t _port;
    v5_device_e_t _deviceType;
};

enum class DeviceType {
    none = 0, motor = 2, rotation = 4, imu = 6, distance = 7,
    radio = 8, vision = 11, adi = 12, optical = 16, gps = 20,
    aivision = 29, serial = 129, undefined = 255
};
} // namespace v5
} // namespace pros
```

---

## 4. Motors

**Headers**: `pros/motors.h`, `pros/motors.hpp`, `pros/abstract_motor.hpp`

### C Enums

```c
typedef enum motor_brake_mode_e {
    E_MOTOR_BRAKE_COAST   = 0,
    E_MOTOR_BRAKE_BRAKE   = 1,
    E_MOTOR_BRAKE_HOLD    = 2,
    E_MOTOR_BRAKE_INVALID = INT32_MAX
} motor_brake_mode_e_t;

typedef enum motor_encoder_units_e {
    E_MOTOR_ENCODER_DEGREES   = 0,   // Position in degrees
    E_MOTOR_ENCODER_ROTATIONS = 1,   // Position in rotations
    E_MOTOR_ENCODER_COUNTS    = 2,   // Position in raw counts
    E_MOTOR_ENCODER_INVALID   = INT32_MAX
} motor_encoder_units_e_t;

typedef enum motor_gearset_e {
    E_MOTOR_GEARSET_36  = 0,  // 36:1, 100RPM (Red cartridge)
    E_MOTOR_GEAR_RED    = 0,
    E_MOTOR_GEAR_100    = 0,
    E_MOTOR_GEARSET_18  = 1,  // 18:1, 200RPM (Green cartridge)
    E_MOTOR_GEAR_GREEN  = 1,
    E_MOTOR_GEAR_200    = 1,
    E_MOTOR_GEARSET_06  = 2,  // 6:1, 600RPM (Blue cartridge)
    E_MOTOR_GEAR_BLUE   = 2,
    E_MOTOR_GEAR_600    = 2,
    E_MOTOR_GEARSET_INVALID = INT32_MAX
} motor_gearset_e_t;

typedef enum motor_type_e {
    E_MOTOR_TYPE_V5  = 0,
    E_MOTOR_TYPE_EXP = 1,
    E_MOTOR_TYPE_INVALID = INT32_MAX
} motor_type_e_t;

typedef enum motor_fault_e {
    E_MOTOR_FAULT_NO_FAULTS       = 0x00,
    E_MOTOR_FAULT_MOTOR_OVER_TEMP = 0x01,
    E_MOTOR_FAULT_DRIVER_FAULT    = 0x02,
    E_MOTOR_FAULT_OVER_CURRENT    = 0x04,
    E_MOTOR_FAULT_DRV_OVER_CURRENT = 0x08
} motor_fault_e_t;

typedef enum motor_flag_e {
    E_MOTOR_FLAGS_NONE          = 0x00,
    E_MOTOR_FLAGS_BUSY          = 0x01,
    E_MOTOR_FLAGS_ZERO_VELOCITY = 0x02,
    E_MOTOR_FLAGS_ZERO_POSITION = 0x04
} motor_flag_e_t;
```

### C Structs
```c
typedef struct motor_pid_full_s {
    uint8_t kf, kp, ki, kd;
    uint8_t filter, limit, threshold, loopspeed;
} motor_pid_full_s_t;

typedef struct motor_pid_s {
    uint8_t kf, kp, ki, kd;
} motor_pid_s_t;
```

### C Functions (all take `int8_t port` as first arg)

**Movement**:
| Function | Returns | Description |
|---|---|---|
| `motor_move(port, voltage)` | `int32_t` | Set voltage -127 to 127 |
| `motor_brake(port)` | `int32_t` | Brake using current brake mode |
| `motor_move_absolute(port, position, velocity)` | `int32_t` | Move to absolute position |
| `motor_move_relative(port, position, velocity)` | `int32_t` | Move relative amount |
| `motor_move_velocity(port, velocity)` | `int32_t` | Set velocity target (RPM) |
| `motor_move_voltage(port, voltage)` | `int32_t` | Set voltage -12000 to 12000 mV |
| `motor_modify_profiled_velocity(port, velocity)` | `int32_t` | Modify in-progress profiled move |

**Target Getters**:
| Function | Returns |
|---|---|
| `motor_get_target_position(port)` | `double` |
| `motor_get_target_velocity(port)` | `int32_t` |

**Telemetry**:
| Function | Returns | Unit |
|---|---|---|
| `motor_get_actual_velocity(port)` | `double` | RPM |
| `motor_get_current_draw(port)` | `int32_t` | mA |
| `motor_get_direction(port)` | `int32_t` | 1 or -1 |
| `motor_get_efficiency(port)` | `double` | % |
| `motor_get_faults(port)` | `uint32_t` | bitmask |
| `motor_get_flags(port)` | `uint32_t` | bitmask |
| `motor_get_position(port)` | `double` | depends on units |
| `motor_get_power(port)` | `double` | Watts |
| `motor_get_raw_position(port, *timestamp)` | `int32_t` | encoder ticks |
| `motor_get_temperature(port)` | `double` | °C |
| `motor_get_torque(port)` | `double` | Nm |
| `motor_get_voltage(port)` | `int32_t` | mV |
| `motor_is_over_current(port)` | `int32_t` | bool |
| `motor_is_over_temp(port)` | `int32_t` | bool |

**Configuration**:
| Function | Returns |
|---|---|
| `motor_set_brake_mode(port, mode)` | `int32_t` |
| `motor_get_brake_mode(port)` | `motor_brake_mode_e_t` |
| `motor_set_current_limit(port, limit)` | `int32_t` |
| `motor_get_current_limit(port)` | `int32_t` |
| `motor_set_encoder_units(port, units)` | `int32_t` |
| `motor_get_encoder_units(port)` | `motor_encoder_units_e_t` |
| `motor_set_gearing(port, gearset)` | `int32_t` |
| `motor_get_gearing(port)` | `motor_gearset_e_t` |
| `motor_set_voltage_limit(port, limit)` | `int32_t` |
| `motor_get_voltage_limit(port)` | `int32_t` |
| `motor_set_zero_position(port, position)` | `int32_t` |
| `motor_tare_position(port)` | `int32_t` |
| `motor_get_type(port)` | `motor_type_e_t` |

### C++ Enums (in `pros` namespace)
```cpp
enum class MotorBrake { coast = 0, brake = 1, hold = 2, invalid = INT32_MAX };
enum class MotorEncoderUnits { degrees = 0, rotations = 1, counts = 2, invalid = INT32_MAX };
// Alias: using MotorUnits = MotorEncoderUnits;
enum class MotorGears { red = 0, green = 1, blue = 2, invalid = INT32_MAX };
// Aliases: MotorGearset, MotorCart, MotorCartridge, MotorGear = MotorGears
enum class MotorType { v5 = 0, exp = 1, invalid = INT32_MAX };
```

### C++ `class AbstractMotor` (pure virtual base)

All methods have both indexed `(index=0)` and `_all()` variants.

**Pure virtual methods**: `move`, `move_absolute`, `move_relative`, `move_velocity`, `move_voltage`, `brake`, `modify_profiled_velocity`, all telemetry getters, all config setters/getters, `set_reversed`, `is_reversed`, `set_zero_position`, `tare_position`, `size`, `get_port`.

### C++ `class Motor : public AbstractMotor, public Device`

```cpp
namespace pros {
inline namespace v5 {
class Motor : public AbstractMotor, public Device {
public:
    // Constructors
    Motor(std::int8_t port);  // negative port = reversed
    Motor(std::int8_t port, MotorGears gearset);
    Motor(std::int8_t port, MotorGears gearset, MotorUnits encoder_units);

    // Movement (all return int32_t, take optional index=0)
    std::int32_t move(std::int32_t voltage, std::uint8_t index = 0);
    std::int32_t move_absolute(double position, std::int32_t velocity, std::uint8_t index = 0);
    std::int32_t move_relative(double position, std::int32_t velocity, std::uint8_t index = 0);
    std::int32_t move_velocity(std::int32_t velocity, std::uint8_t index = 0);
    std::int32_t move_voltage(std::int32_t voltage, std::uint8_t index = 0);
    std::int32_t brake(std::uint8_t index = 0);
    std::int32_t modify_profiled_velocity(std::int32_t velocity, std::uint8_t index = 0);

    // Telemetry (all with index=0 default, return appropriate type)
    double get_target_position(std::uint8_t index = 0) const;
    std::int32_t get_target_velocity(std::uint8_t index = 0) const;
    double get_actual_velocity(std::uint8_t index = 0) const;
    std::int32_t get_current_draw(std::uint8_t index = 0) const;
    std::int32_t get_direction(std::uint8_t index = 0) const;
    double get_efficiency(std::uint8_t index = 0) const;
    std::uint32_t get_faults(std::uint8_t index = 0) const;
    std::uint32_t get_flags(std::uint8_t index = 0) const;
    double get_position(std::uint8_t index = 0) const;
    double get_power(std::uint8_t index = 0) const;
    std::int32_t get_raw_position(std::uint32_t* timestamp, std::uint8_t index = 0) const;
    double get_temperature(std::uint8_t index = 0) const;
    double get_torque(std::uint8_t index = 0) const;
    std::int32_t get_voltage(std::uint8_t index = 0) const;
    std::int32_t is_over_current(std::uint8_t index = 0) const;
    std::int32_t is_over_temp(std::uint8_t index = 0) const;

    // Configuration
    std::int32_t set_brake_mode(MotorBrake mode, std::uint8_t index = 0);
    std::int32_t set_brake_mode(motor_brake_mode_e_t mode, std::uint8_t index = 0);
    MotorBrake get_brake_mode(std::uint8_t index = 0) const;
    std::int32_t set_current_limit(std::int32_t limit, std::uint8_t index = 0);
    std::int32_t get_current_limit(std::uint8_t index = 0) const;
    std::int32_t set_encoder_units(MotorUnits units, std::uint8_t index = 0);
    std::int32_t set_encoder_units(motor_encoder_units_e_t units, std::uint8_t index = 0);
    MotorUnits get_encoder_units(std::uint8_t index = 0) const;
    std::int32_t set_gearing(MotorGears gears, std::uint8_t index = 0);
    std::int32_t set_gearing(motor_gearset_e_t gears, std::uint8_t index = 0);
    MotorGears get_gearing(std::uint8_t index = 0) const;
    std::int32_t set_voltage_limit(std::int32_t limit, std::uint8_t index = 0);
    std::int32_t get_voltage_limit(std::uint8_t index = 0) const;
    std::int32_t set_reversed(bool reverse, std::uint8_t index = 0);
    std::int32_t is_reversed(std::uint8_t index = 0) const;
    std::int32_t set_zero_position(double position, std::uint8_t index = 0);
    std::int32_t tare_position(std::uint8_t index = 0);
    MotorType get_type(std::uint8_t index = 0) const;

    // All _all() variants return std::vector<T> of single-element
    // (e.g. get_position_all() -> std::vector<double>)

    std::int8_t size() const;  // always returns 1
    std::int8_t get_port(std::uint8_t index = 0) const;
    static std::vector<Motor> get_all_devices();

private:
    std::int8_t _port;
};

// Literals
const Motor operator"" _mtr(unsigned long long int port);    // e.g. 1_mtr
const Motor operator"" _rmtr(unsigned long long int port);   // reversed motor
} // namespace v5
} // namespace pros
```

---

## 5. Motor Groups

**Header**: `pros/motor_group.hpp`

```cpp
namespace pros {
inline namespace v5 {
class MotorGroup : public virtual AbstractMotor {
public:
    // Constructors
    MotorGroup(std::initializer_list<std::int8_t> ports);
    MotorGroup(const std::vector<std::int8_t> ports, MotorGears gearset = MotorGears::invalid,
               MotorUnits encoder_units = MotorUnits::invalid);
    MotorGroup(AbstractMotor& motor);

    // All same movement/telemetry/config methods as Motor
    // Each has indexed (index=0) and _all() variants
    // Commands apply to all motors in the group

    // Port Management
    void append(AbstractMotor& motor);
    void erase_port(std::int8_t port);
    MotorGroup& operator+=(AbstractMotor& other);
    std::int8_t size() const;
    std::int8_t get_port(std::uint8_t index = 0) const;
    std::vector<std::int8_t> get_port_all() const;

private:
    std::vector<std::int8_t> _ports;
    mutable pros::Mutex _MotorGroup_mutex;
};
} // namespace v5
} // namespace pros
```

---

## 6. Controller & Misc

**Headers**: `pros/misc.h`, `pros/misc.hpp`

### C Enums
```c
typedef enum controller_id_e {
    E_CONTROLLER_MASTER  = 0,
    E_CONTROLLER_PARTNER = 1
} controller_id_e_t;

typedef enum controller_analog_e {
    E_CONTROLLER_ANALOG_LEFT_X  = 0,
    E_CONTROLLER_ANALOG_LEFT_Y  = 1,
    E_CONTROLLER_ANALOG_RIGHT_X = 2,
    E_CONTROLLER_ANALOG_RIGHT_Y = 3
} controller_analog_e_t;

typedef enum controller_digital_e {
    E_CONTROLLER_DIGITAL_L1    = 6,
    E_CONTROLLER_DIGITAL_L2    = 7,
    E_CONTROLLER_DIGITAL_R1    = 8,
    E_CONTROLLER_DIGITAL_R2    = 9,
    E_CONTROLLER_DIGITAL_UP    = 10,
    E_CONTROLLER_DIGITAL_DOWN  = 11,
    E_CONTROLLER_DIGITAL_LEFT  = 12,
    E_CONTROLLER_DIGITAL_RIGHT = 13,
    E_CONTROLLER_DIGITAL_X     = 14,
    E_CONTROLLER_DIGITAL_B     = 15,
    E_CONTROLLER_DIGITAL_Y     = 16,
    E_CONTROLLER_DIGITAL_A     = 17,
    E_CONTROLLER_DIGITAL_POWER = 18  // virtual, not physical button
} controller_digital_e_t;
```

### C Functions — Controller
| Function | Returns | Description |
|---|---|---|
| `controller_is_connected(id)` | `int32_t` | 1 if connected |
| `controller_get_analog(id, channel)` | `int32_t` | -127 to 127 |
| `controller_get_digital(id, button)` | `int32_t` | 1 if pressed |
| `controller_get_digital_new_press(id, button)` | `int32_t` | Rising edge |
| `controller_get_battery_capacity(id)` | `int32_t` | Battery % |
| `controller_get_battery_level(id)` | `int32_t` | Battery mV |
| `controller_print(id, line, col, fmt, ...)` | `int32_t` | Print to controller |
| `controller_set_text(id, line, col, str)` | `int32_t` | Set text |
| `controller_clear_line(id, line)` | `int32_t` | Clear line 0-2 |
| `controller_clear(id)` | `int32_t` | Clear all |
| `controller_rumble(id, pattern)` | `int32_t` | Rumble pattern |

### C Functions — Competition
| Function | Returns | Description |
|---|---|---|
| `competition_get_status()` | `uint8_t` | Status bitmask |
| `competition_is_disabled()` | `uint8_t` | True if disabled |
| `competition_is_connected()` | `uint8_t` | True if field control |
| `competition_is_autonomous()` | `uint8_t` | True if auton |
| `competition_is_field()` | `uint8_t` | True if field controller |
| `competition_is_switch()` | `uint8_t` | True if competition switch |

**Competition status bits**: `COMPETITION_DISABLED = 1<<0`, `COMPETITION_AUTONOMOUS = 1<<1`, `COMPETITION_CONNECTED = 1<<2`, `COMPETITION_SYSTEM = 1<<3`

### C Functions — Battery
| Function | Returns | Unit |
|---|---|---|
| `battery_get_voltage()` | `int32_t` | mV |
| `battery_get_current()` | `int32_t` | mA |
| `battery_get_temperature()` | `double` | °C |
| `battery_get_capacity()` | `double` | % |

### C Functions — USD (MicroSD)
| Function | Returns |
|---|---|
| `usd_is_installed()` | `int32_t` |
| `usd_list_files(path)` | `int32_t` |

### Constants
```c
#define NUM_V5_PORTS 22
```

### C++ Classes

```cpp
namespace pros {
class Controller {
public:
    Controller(controller_id_e_t id);

    std::int32_t is_connected();
    std::int32_t get_analog(controller_analog_e_t channel);
    std::int32_t get_digital(controller_digital_e_t button);
    std::int32_t get_digital_new_press(controller_digital_e_t button);
    std::int32_t get_battery_capacity();
    std::int32_t get_battery_level();
    template <typename... Params>
    std::int32_t print(std::uint8_t line, std::uint8_t col, const char* fmt, Params... args);
    std::int32_t set_text(std::uint8_t line, std::uint8_t col, const char* str);
    std::int32_t set_text(std::uint8_t line, std::uint8_t col, const std::string& str);
    std::int32_t clear_line(std::uint8_t line);
    std::int32_t clear();
    std::int32_t rumble(const char* pattern);
    // Operator overloads for analog/digital access
    std::int32_t operator[](controller_analog_e_t channel);
    std::int32_t operator[](controller_digital_e_t button);
private:
    controller_id_e_t _id;
};

namespace battery {
    std::int32_t get_voltage();
    std::int32_t get_current();
    double get_temperature();
    double get_capacity();
}

namespace competition {
    std::uint8_t get_status();
    bool is_disabled();
    bool is_connected();
    bool is_autonomous();
    bool is_field();
    bool is_switch();
}

namespace usd {
    bool is_installed();
}
} // namespace pros
```

---

## 7. ADI (3-Wire)

**Headers**: `pros/adi.h`, `pros/adi.hpp`

### C Enum
```c
typedef enum adi_port_config_e {
    E_ADI_ANALOG_IN     = 0,
    E_ADI_ANALOG_OUT    = 1,
    E_ADI_DIGITAL_IN    = 2,
    E_ADI_DIGITAL_OUT   = 3,
    // Legacy types
    E_ADI_LEGACY_GYRO        = 10,
    E_ADI_LEGACY_SERVO       = 12,
    E_ADI_LEGACY_PWM         = 13,
    E_ADI_LEGACY_ENCODER     = 14,
    E_ADI_LEGACY_ULTRASONIC  = 15,
    E_ADI_TYPE_UNDEFINED     = 255
} adi_port_config_e_t;

typedef enum adi_potentiometer_type_e {
    E_ADI_POT_EDR = 0,
    E_ADI_POT_V2  = 1
} adi_potentiometer_type_e_t;
```

### C Constants
```c
#define INTERNAL_ADI_PORT  22
#define NUM_ADI_PORTS       8
#define HIGH                1
#define LOW                 0
#define INPUT            0x00
#define OUTPUT           0x01
#define INPUT_ANALOG     0x02
#define OUTPUT_ANALOG    0x03
```

### C Functions — Basic ADI
| Function | Returns |
|---|---|
| `adi_port_set_config(port, type)` | `int32_t` |
| `adi_port_get_config(port)` | `adi_port_config_e_t` |
| `adi_port_set_value(port, value)` | `int32_t` |
| `adi_port_get_value(port)` | `int32_t` |
| `adi_analog_read(port)` | `int32_t` (0-4095) |
| `adi_analog_calibrate(port)` | `int32_t` |
| `adi_analog_read_calibrated(port)` | `int32_t` (-4095 to 4095) |
| `adi_analog_read_calibrated_HR(port)` | `int32_t` (-16384 to 16384) |
| `adi_digital_read(port)` | `int32_t` |
| `adi_digital_write(port, value)` | `int32_t` |
| `adi_digital_get_new_press(port)` | `int32_t` |
| `adi_pin_mode(port, mode)` | `int32_t` |

### C Functions — ADI Motor
| Function | Returns |
|---|---|
| `adi_motor_set(port, speed)` | `int32_t` |
| `adi_motor_get(port)` | `int32_t` |
| `adi_motor_stop(port)` | `int32_t` |

### C Functions — ADI Encoder
```c
typedef int32_t adi_encoder_t;
adi_encoder_t adi_encoder_init(uint8_t port_top, uint8_t port_bottom, bool reverse);
int32_t adi_encoder_get(adi_encoder_t enc);     // 360 ticks/rev
int32_t adi_encoder_reset(adi_encoder_t enc);
int32_t adi_encoder_shutdown(adi_encoder_t enc);
```

### C Functions — ADI Ultrasonic
```c
typedef int32_t adi_ultrasonic_t;
adi_ultrasonic_t adi_ultrasonic_init(uint8_t port_ping, uint8_t port_echo);
int32_t adi_ultrasonic_get(adi_ultrasonic_t ult);  // centimeters
int32_t adi_ultrasonic_shutdown(adi_ultrasonic_t ult);
```

### C Functions — ADI Gyro
```c
typedef int32_t adi_gyro_t;
adi_gyro_t adi_gyro_init(uint8_t port, double multiplier);
double adi_gyro_get(adi_gyro_t gyro);  // tenths of degrees
int32_t adi_gyro_reset(adi_gyro_t gyro);
int32_t adi_gyro_shutdown(adi_gyro_t gyro);
```

### C Functions — ADI Potentiometer
```c
typedef int32_t adi_potentiometer_t;
adi_potentiometer_t adi_potentiometer_init(uint8_t port);
adi_potentiometer_t adi_potentiometer_type_init(uint8_t port, adi_potentiometer_type_e_t type);
double adi_potentiometer_get_angle(adi_potentiometer_t potentiometer);
```

### C Functions — ADI LED
```c
typedef int32_t adi_led_t;
adi_led_t adi_led_init(uint8_t port);
int32_t adi_led_clear_all(adi_led_t led, uint32_t* buffer, uint32_t buffer_length);
int32_t adi_led_set(adi_led_t led, uint32_t* buffer, uint32_t buffer_length);
int32_t adi_led_set_all(adi_led_t led, uint32_t* buffer, uint32_t buffer_length, uint32_t color);
int32_t adi_led_set_pixel(adi_led_t led, uint32_t* buffer, uint32_t buffer_length, uint32_t color, uint32_t pixel_position);
int32_t adi_led_clear_pixel(adi_led_t led, uint32_t* buffer, uint32_t buffer_length, uint32_t pixel_position);
```

### C++ Classes (namespace `pros::adi`)

```cpp
namespace pros::adi {

// Type aliases for expander ports
using ext_adi_port_pair_t = std::pair<uint8_t, uint8_t>;     // {smart_port, adi_port}
using ext_adi_port_tuple_t = std::tuple<uint8_t, uint8_t, uint8_t>; // {smart, adi1, adi2}

class Port {
public:
    Port(uint8_t adi_port, adi_port_config_e_t type = E_ADI_TYPE_UNDEFINED);
    Port(ext_adi_port_pair_t port_pair, adi_port_config_e_t type = E_ADI_TYPE_UNDEFINED);
    int32_t set_config(adi_port_config_e_t type);
    adi_port_config_e_t get_config() const;
    int32_t set_value(int32_t value);
    int32_t get_value() const;
    uint8_t get_port() const;
};

class AnalogIn : private Port {
public:
    AnalogIn(uint8_t adi_port);
    AnalogIn(ext_adi_port_pair_t port_pair);
    int32_t calibrate();
    int32_t get_value_calibrated();
    int32_t get_value_calibrated_HR();
    using Port::get_value;
    using Port::get_port;
};
using LineSensor = AnalogIn;
using LightSensor = AnalogIn;
using Accelerometer = AnalogIn;

class AnalogOut : private Port {
public:
    AnalogOut(uint8_t adi_port);
    AnalogOut(ext_adi_port_pair_t port_pair);
    using Port::set_value;
    using Port::get_port;
};

class DigitalOut : private Port {
public:
    DigitalOut(uint8_t adi_port, bool init_state = LOW);
    DigitalOut(ext_adi_port_pair_t port_pair, bool init_state = LOW);
    using Port::set_value;
    using Port::get_port;
};

class DigitalIn : private Port {
public:
    DigitalIn(uint8_t adi_port);
    DigitalIn(ext_adi_port_pair_t port_pair);
    int32_t get_new_press();
    using Port::get_value;
    using Port::get_port;
};
using Button = DigitalIn;

class Motor : private Port {
public:
    Motor(uint8_t adi_port);
    Motor(ext_adi_port_pair_t port_pair);
    int32_t stop();
    using Port::set_value;
    using Port::get_value;
    using Port::get_port;
};

class Encoder : private Port {
public:
    Encoder(uint8_t adi_port_top, uint8_t adi_port_bottom, bool reversed = false);
    Encoder(ext_adi_port_tuple_t port_tuple, bool reversed = false);
    int32_t reset();
    int32_t get_value() const;
private:
    ext_adi_port_pair_t _port_pair;
};

class Ultrasonic : private Port {
public:
    Ultrasonic(uint8_t adi_port_ping, uint8_t adi_port_echo);
    Ultrasonic(ext_adi_port_tuple_t port_tuple);
    int32_t get_value() const;
};

class Gyro : private Port {
public:
    Gyro(uint8_t adi_port, double multiplier = 1.0);
    Gyro(ext_adi_port_pair_t port_pair, double multiplier = 1.0);
    double get_value() const;
    int32_t reset();
};

class Potentiometer : public AnalogIn {
public:
    Potentiometer(uint8_t adi_port, adi_potentiometer_type_e_t type = E_ADI_POT_EDR);
    Potentiometer(ext_adi_port_pair_t port_pair, adi_potentiometer_type_e_t type = E_ADI_POT_EDR);
    double get_angle() const;
    using AnalogIn::calibrate;
    using AnalogIn::get_value_calibrated;
};

class Led : protected Port {
public:
    Led(uint8_t adi_port, uint32_t length);
    Led(ext_adi_port_pair_t port_pair, uint32_t length);
    uint32_t& operator[](uint32_t i);
    int32_t clear_all();
    int32_t clear();
    int32_t update();
    int32_t set_all(uint32_t color);
    int32_t set_pixel(uint32_t color, uint32_t pos);
    int32_t clear_pixel(uint32_t pos);
    uint32_t length() const;
protected:
    std::vector<uint32_t> _buffer;
};
using LED = Led;

class Pneumatics : public DigitalOut {
public:
    Pneumatics(uint8_t adi_port, bool start_extended, bool extended_is_low = false);
    Pneumatics(ext_adi_port_pair_t port_pair, bool start_extended, bool extended_is_low = false);
    int32_t extend();
    int32_t retract();
    int32_t toggle();
    bool is_extended() const;
private:
    bool state;
    bool extended_is_low;
};

// Legacy typedefs
using ADIPort = Port;
using ADIAnalogIn = AnalogIn;
using ADIAnalogOut = AnalogOut;
using ADIDigitalIn = DigitalIn;
using ADIDigitalOut = DigitalOut;
using ADIMotor = Motor;
using ADIEncoder = Encoder;
using ADIUltrasonic = Ultrasonic;
using ADIGyro = Gyro;
using ADIPotentiometer = Potentiometer;
using ADILed = Led;
using ADIPneumatics = Pneumatics;

} // namespace pros::adi
```

---

## 8. ADI Expander (ext_adi)

**Header**: `pros/ext_adi.h`

Mirror of all `adi_*` C functions but with `uint8_t smart_port` as the first parameter (the smart port 1-21 where the ADI Expander is plugged in).

| Function | Returns |
|---|---|
| `ext_adi_port_get_config(smart, adi)` | `adi_port_config_e_t` |
| `ext_adi_port_get_value(smart, adi)` | `int32_t` |
| `ext_adi_port_set_config(smart, adi, type)` | `int32_t` |
| `ext_adi_port_set_value(smart, adi, value)` | `int32_t` |
| `ext_adi_analog_calibrate(smart, adi)` | `int32_t` |
| `ext_adi_analog_read(smart, adi)` | `int32_t` |
| `ext_adi_analog_read_calibrated(smart, adi)` | `int32_t` |
| `ext_adi_analog_read_calibrated_HR(smart, adi)` | `int32_t` |
| `ext_adi_digital_read(smart, adi)` | `int32_t` |
| `ext_adi_digital_get_new_press(smart, adi)` | `int32_t` |
| `ext_adi_digital_write(smart, adi, value)` | `int32_t` |
| `ext_adi_pin_mode(smart, adi, mode)` | `int32_t` |
| `ext_adi_motor_set(smart, adi, speed)` | `int32_t` |
| `ext_adi_motor_get(smart, adi)` | `int32_t` |
| `ext_adi_motor_stop(smart, adi)` | `int32_t` |

**Ext ADI compound devices**:
```c
typedef int32_t ext_adi_encoder_t;
ext_adi_encoder_t ext_adi_encoder_init(uint8_t smart, uint8_t top, uint8_t bottom, bool reverse);
int32_t ext_adi_encoder_get(ext_adi_encoder_t enc);
int32_t ext_adi_encoder_reset(ext_adi_encoder_t enc);
int32_t ext_adi_encoder_shutdown(ext_adi_encoder_t enc);

typedef int32_t ext_adi_ultrasonic_t;
ext_adi_ultrasonic_t ext_adi_ultrasonic_init(uint8_t smart, uint8_t ping, uint8_t echo);
int32_t ext_adi_ultrasonic_get(ext_adi_ultrasonic_t ult);
int32_t ext_adi_ultrasonic_shutdown(ext_adi_ultrasonic_t ult);

typedef int32_t ext_adi_gyro_t;
ext_adi_gyro_t ext_adi_gyro_init(uint8_t smart, uint8_t adi, double multiplier);
double ext_adi_gyro_get(ext_adi_gyro_t gyro);
int32_t ext_adi_gyro_reset(ext_adi_gyro_t gyro);
int32_t ext_adi_gyro_shutdown(ext_adi_gyro_t gyro);

typedef int32_t ext_adi_potentiometer_t;
ext_adi_potentiometer_t ext_adi_potentiometer_init(uint8_t smart, uint8_t adi, adi_potentiometer_type_e_t type);
double ext_adi_potentiometer_get_angle(ext_adi_potentiometer_t pot);

typedef int32_t ext_adi_led_t;
ext_adi_led_t ext_adi_led_init(uint8_t smart, uint8_t adi);
int32_t ext_adi_led_clear_all(ext_adi_led_t led, uint32_t* buf, uint32_t len);
int32_t ext_adi_led_set(ext_adi_led_t led, uint32_t* buf, uint32_t len);
int32_t ext_adi_led_set_all(ext_adi_led_t led, uint32_t* buf, uint32_t len, uint32_t color);
int32_t ext_adi_led_set_pixel(ext_adi_led_t led, uint32_t* buf, uint32_t len, uint32_t color, uint32_t pos);
int32_t ext_adi_led_clear_pixel(ext_adi_led_t led, uint32_t* buf, uint32_t len, uint32_t pos);
```

> **Note**: The C++ ADI classes already support expander ports via `ext_adi_port_pair_t` and `ext_adi_port_tuple_t` constructor overloads, so there is no separate C++ ext_adi class.

---

## 9. IMU (Inertial)

**Headers**: `pros/imu.h`, `pros/imu.hpp`

### C Enums
```c
typedef enum imu_status_e {
    E_IMU_STATUS_READY      = 0,
    E_IMU_STATUS_CALIBRATING = 1,
    E_IMU_STATUS_ERROR       = 0xFF
} imu_status_e_t;

typedef enum imu_orientation_e {
    E_IMU_Z_UP    = 0,
    E_IMU_Z_DOWN  = 1,
    E_IMU_X_UP    = 2,
    E_IMU_X_DOWN  = 3,
    E_IMU_Y_UP    = 4,
    E_IMU_Y_DOWN  = 5
} imu_orientation_e_t;
```

### C Structs
```c
typedef struct quaternion_s {
    double x, y, z, w;
} quaternion_s_t;

typedef struct imu_raw_s {  // also: imu_gyro_s_t, imu_accel_s_t
    double x, y, z;
} imu_raw_s, imu_gyro_s_t, imu_accel_s_t;

typedef struct euler_s {
    double pitch, roll, yaw;
} euler_s_t;
```

### C Functions
| Function | Returns | Unit |
|---|---|---|
| `imu_reset(port)` | `int32_t` | Recalibrate (~2s) |
| `imu_set_data_rate(port, rate)` | `int32_t` | Rate in ms (min 5) |
| `imu_get_rotation(port)` | `double` | degrees (unbounded) |
| `imu_get_heading(port)` | `double` | 0-360 degrees |
| `imu_get_quaternion(port)` | `quaternion_s_t` | |
| `imu_get_euler(port)` | `euler_s_t` | |
| `imu_get_pitch(port)` | `double` | degrees |
| `imu_get_roll(port)` | `double` | degrees |
| `imu_get_yaw(port)` | `double` | degrees |
| `imu_get_gyro_rate(port)` | `imu_gyro_s_t` | °/s |
| `imu_get_accel(port)` | `imu_accel_s_t` | g |
| `imu_get_status(port)` | `imu_status_e_t` | |
| `imu_get_physical_orientation(port)` | `imu_orientation_e_t` | |
| `imu_tare_heading(port)` | `int32_t` | |
| `imu_tare_rotation(port)` | `int32_t` | |
| `imu_tare_pitch(port)` | `int32_t` | |
| `imu_tare_roll(port)` | `int32_t` | |
| `imu_tare_yaw(port)` | `int32_t` | |
| `imu_tare_euler(port)` | `int32_t` | Tare pitch+roll+yaw |
| `imu_tare(port)` | `int32_t` | Tare heading+rotation+euler |
| `imu_set_heading(port, target)` | `int32_t` | |
| `imu_set_rotation(port, target)` | `int32_t` | |
| `imu_set_pitch(port, target)` | `int32_t` | |
| `imu_set_roll(port, target)` | `int32_t` | |
| `imu_set_yaw(port, target)` | `int32_t` | |
| `imu_set_euler(port, target)` | `int32_t` | |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Imu : public Device {
public:
    Imu(uint8_t port);

    // Core
    std::int32_t reset();
    std::int32_t set_data_rate(std::uint32_t rate);
    bool is_calibrating() const;

    // Getters
    double get_rotation() const;
    double get_heading() const;
    quaternion_s_t get_quaternion() const;
    euler_s_t get_euler() const;
    double get_pitch() const;
    double get_roll() const;
    double get_yaw() const;
    imu_gyro_s_t get_gyro_rate() const;
    imu_accel_s_t get_accel() const;
    imu_status_e_t get_status() const;
    imu_orientation_e_t get_physical_orientation() const;

    // Tare (set to zero)
    std::int32_t tare_heading();
    std::int32_t tare_rotation();
    std::int32_t tare_pitch();
    std::int32_t tare_roll();
    std::int32_t tare_yaw();
    std::int32_t tare_euler();
    std::int32_t tare();

    // Set specific values
    std::int32_t set_heading(double target);
    std::int32_t set_rotation(double target);
    std::int32_t set_pitch(double target);
    std::int32_t set_roll(double target);
    std::int32_t set_yaw(double target);
    std::int32_t set_euler(euler_s_t target);
};

using IMU = Imu;
const Imu operator"" _imu(unsigned long long int port);  // e.g. 1_imu
} // namespace v5
} // namespace pros
```

---

## 10. Rotation Sensor

**Headers**: `pros/rotation.h`, `pros/rotation.hpp`

### C Constant
```c
#define ROTATION_MINIMUM_DATA_RATE 5  // ms
```

### C Functions
| Function | Returns | Unit |
|---|---|---|
| `rotation_reset(port)` | `int32_t` | Reset to 0 |
| `rotation_set_data_rate(port, rate)` | `int32_t` | ms (min 5) |
| `rotation_set_position(port, position)` | `int32_t` | centidegrees |
| `rotation_reset_position(port)` | `int32_t` | Reset to 0 |
| `rotation_get_position(port)` | `int32_t` | centidegrees |
| `rotation_get_velocity(port)` | `int32_t` | centideg/s |
| `rotation_get_angle(port)` | `int32_t` | 0-36000 centideg |
| `rotation_set_reversed(port, value)` | `int32_t` | |
| `rotation_reverse(port)` | `int32_t` | Toggle |
| `rotation_get_reversed(port)` | `int32_t` | |
| `rotation_init_reverse(port, reversed)` | `int32_t` | Init & set |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Rotation : public Device {
public:
    Rotation(uint8_t port);
    Rotation(uint8_t port, bool reversed);

    std::int32_t reset();
    std::int32_t set_data_rate(std::uint32_t rate);
    std::int32_t set_position(std::uint32_t position);
    std::int32_t reset_position();
    std::int32_t get_position() const;
    std::int32_t get_velocity() const;
    std::int32_t get_angle() const;
    std::int32_t set_reversed(bool value);
    std::int32_t reverse();
    std::int32_t get_reversed() const;
    static std::vector<Rotation> get_all_devices();
};

const Rotation operator"" _rot(unsigned long long int port);
} // namespace v5
} // namespace pros
```

---

## 11. Distance Sensor

**Headers**: `pros/distance.h`, `pros/distance.hpp`

### C Functions
| Function | Returns | Unit |
|---|---|---|
| `distance_get(port)` | `int32_t` | mm |
| `distance_get_confidence(port)` | `int32_t` | 0-100 |
| `distance_get_object_size(port)` | `int32_t` | relative (0-400) |
| `distance_get_object_velocity(port)` | `double` | m/s (approach +, retreat -) |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Distance : public Device {
public:
    Distance(uint8_t port);

    std::int32_t get() const;         // alias for get_distance
    std::int32_t get_distance() const;
    std::int32_t get_confidence() const;
    std::int32_t get_object_size() const;
    double get_object_velocity() const;
    static std::vector<Distance> get_all_devices();
};
} // namespace v5
} // namespace pros
```

---

## 12. Optical Sensor

**Headers**: `pros/optical.h`, `pros/optical.hpp`

### C Enums & Structs
```c
typedef enum optical_direction_e {
    E_OPTICAL_DIRECTION_NO_GESTURE = 0,
    E_OPTICAL_DIRECTION_UP    = 1,
    E_OPTICAL_DIRECTION_DOWN  = 2,
    E_OPTICAL_DIRECTION_RIGHT = 3,
    E_OPTICAL_DIRECTION_LEFT  = 4
} optical_direction_e_t;

typedef struct optical_rgb_s {
    double red, green, blue, brightness;
} optical_rgb_s_t;

typedef struct optical_raw_s {
    uint32_t clear, red, green, blue;
} optical_raw_s_t;

typedef struct optical_gesture_s {
    uint8_t udata, ddata, ldata, rdata;
    uint8_t type;
    uint8_t pad;
    uint16_t count;
    uint32_t time;
} optical_gesture_s_t;
```

### C Functions
| Function | Returns | Unit |
|---|---|---|
| `optical_get_hue(port)` | `double` | 0-360 |
| `optical_get_saturation(port)` | `double` | 0-1.0 |
| `optical_get_brightness(port)` | `double` | 0-1.0 |
| `optical_get_proximity(port)` | `int32_t` | 0-255 |
| `optical_set_led_pwm(port, value)` | `int32_t` | 0-100 |
| `optical_get_led_pwm(port)` | `int32_t` | 0-100 |
| `optical_get_rgb(port)` | `optical_rgb_s_t` | |
| `optical_get_raw(port)` | `optical_raw_s_t` | |
| `optical_get_gesture(port)` | `optical_gesture_s_t` | |
| `optical_get_gesture_raw(port)` | `optical_gesture_s_t` | |
| `optical_enable_gesture(port)` | `int32_t` | |
| `optical_disable_gesture(port)` | `int32_t` | |
| `optical_set_integration_time(port, time)` | `int32_t` | |
| `optical_get_integration_time(port)` | `double` | |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Optical : public Device {
public:
    Optical(uint8_t port);

    double get_hue() const;
    double get_saturation() const;
    double get_brightness() const;
    std::int32_t get_proximity() const;
    std::int32_t set_led_pwm(uint8_t value);
    std::int32_t get_led_pwm() const;
    optical_rgb_s_t get_rgb() const;
    optical_raw_s_t get_raw() const;
    optical_gesture_s_t get_gesture() const;
    optical_gesture_s_t get_gesture_raw() const;
    std::int32_t enable_gesture() const;
    std::int32_t disable_gesture() const;
    std::int32_t set_integration_time(double time);
    double get_integration_time() const;
    static std::vector<Optical> get_all_devices();
};

const Optical operator"" _opt(unsigned long long int port);
} // namespace v5
} // namespace pros
```

---

## 13. GPS Sensor

**Headers**: `pros/gps.h`, `pros/gps.hpp`

### C Structs
```c
typedef struct gps_position_s {
    double x, y;
} gps_position_s_t;

typedef struct gps_status_s {
    double x, y, pitch, roll, yaw;
} gps_status_s_t;

typedef struct gps_orientation_s {
    double pitch, roll, yaw;
} gps_orientation_s_t;

typedef struct __attribute__((packed)) gps_raw_s {
    double x, y, z;
} gps_raw_s, gps_accel_s_t, gps_gyro_s_t;
```

### C Functions
| Function | Returns |
|---|---|
| `gps_initialize_full(port, xInitial, yInitial, headingInitial, xOffset, yOffset)` | `int32_t` |
| `gps_set_offset(port, xOffset, yOffset)` | `int32_t` |
| `gps_get_offset(port)` | `gps_position_s_t` |
| `gps_set_position(port, xInitial, yInitial, headingInitial)` | `int32_t` |
| `gps_set_data_rate(port, rate)` | `int32_t` |
| `gps_get_error(port)` | `double` |
| `gps_get_position_and_orientation(port)` | `gps_status_s_t` |
| `gps_get_position(port)` | `gps_position_s_t` |
| `gps_get_position_x(port)` | `double` |
| `gps_get_position_y(port)` | `double` |
| `gps_get_orientation(port)` | `gps_orientation_s_t` |
| `gps_get_pitch(port)` | `double` |
| `gps_get_roll(port)` | `double` |
| `gps_get_yaw(port)` | `double` |
| `gps_get_heading(port)` | `double` |
| `gps_get_heading_raw(port)` | `double` |
| `gps_get_gyro_rate(port)` | `gps_gyro_s_t` |
| `gps_get_gyro_rate_x/y/z(port)` | `double` |
| `gps_get_accel(port)` | `gps_accel_s_t` |
| `gps_get_accel_x/y/z(port)` | `double` |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Gps : public Device {
public:
    Gps(uint8_t port);
    Gps(uint8_t port, double xInitial, double yInitial, double headingInitial);
    Gps(uint8_t port, double xOffset, double yOffset);
    Gps(uint8_t port, double xInitial, double yInitial, double headingInitial,
        double xOffset, double yOffset);

    // All matching C API methods
    std::int32_t initialize_full(double xInitial, double yInitial, double headingInitial,
                                  double xOffset, double yOffset);
    std::int32_t set_offset(double xOffset, double yOffset);
    gps_position_s_t get_offset() const;
    std::int32_t set_position(double xInitial, double yInitial, double headingInitial);
    std::int32_t set_data_rate(std::uint32_t rate);
    double get_error() const;
    gps_status_s_t get_position_and_orientation() const;
    gps_position_s_t get_position() const;
    double get_position_x() const;
    double get_position_y() const;
    gps_orientation_s_t get_orientation() const;
    double get_pitch() const;
    double get_roll() const;
    double get_yaw() const;
    double get_heading() const;
    double get_heading_raw() const;
    gps_gyro_s_t get_gyro_rate() const;
    double get_gyro_rate_x() const;
    double get_gyro_rate_y() const;
    double get_gyro_rate_z() const;
    gps_accel_s_t get_accel() const;
    double get_accel_x() const;
    double get_accel_y() const;
    double get_accel_z() const;

    friend std::ostream& operator<<(std::ostream&, const Gps&);
    static std::vector<Gps> get_gps();
};

using GPS = Gps;
const Gps operator"" _gps(unsigned long long int port);
} // namespace v5
} // namespace pros
```

---

## 14. Vision Sensor

**Headers**: `pros/vision.h`, `pros/vision.hpp`

### C Enums & Types
```c
typedef enum vision_object_type_e {
    E_VISION_OBJECT_NORMAL     = 0,
    E_VISION_OBJECT_COLOR_CODE = 1,
    E_VISION_OBJECT_LINE       = 2
} vision_object_type_e_t;

typedef enum vision_zero_e {
    E_VISION_ZERO_TOPLEFT = 0,
    E_VISION_ZERO_CENTER  = 1
} vision_zero_e_t;

typedef uint16_t vision_color_code_t;
```

### C Structs
```c
typedef struct vision_signature {
    uint8_t id;
    uint8_t _pad[3];
    float range;
    int32_t u_min, u_max, u_mean;
    int32_t v_min, v_max, v_mean;
    uint32_t rgb;
    uint32_t type;
} vision_signature_s_t;

typedef struct vision_object {
    vision_object_type_e_t type;   // NORMAL, COLOR_CODE, or LINE
    int16_t left_coord, top_coord;
    int16_t width, height;
    uint16_t x_middle_coord, y_middle_coord;
    uint16_t signature;            // sig# or color_code_t
    double angle;
} vision_object_s_t;
```

### C Constants
```c
#define VISION_OBJECT_ERR_SIG 255
#define VISION_FOV_WIDTH      316   // pixels
#define VISION_FOV_HEIGHT     212   // pixels
```

### C Functions
| Function | Returns |
|---|---|
| `vision_clear_led(port)` | `int32_t` |
| `vision_signature_from_utility(id, u_min, u_max, u_mean, v_min, v_max, v_mean, range, type)` | `vision_signature_s_t` |
| `vision_create_color_code(port, sig_id1, sig_id2, ...)` | `vision_color_code_t` |
| `vision_get_by_size(port, size_id)` | `vision_object_s_t` |
| `vision_get_by_sig(port, size_id, sig_id)` | `vision_object_s_t` |
| `vision_get_by_code(port, size_id, color_code)` | `vision_object_s_t` |
| `vision_get_exposure(port)` | `int32_t` |
| `vision_get_object_count(port)` | `int32_t` |
| `vision_get_white_balance(port)` | `int32_t` |
| `vision_read_by_size(port, size_id, count, *arr)` | `int32_t` |
| `vision_read_by_sig(port, size_id, sig_id, count, *arr)` | `int32_t` |
| `vision_read_by_code(port, size_id, code, count, *arr)` | `int32_t` |
| `vision_get_signature(port, sig_id)` | `vision_signature_s_t` |
| `vision_set_signature(port, sig_id, *sig)` | `int32_t` |
| `vision_set_auto_white_balance(port, enable)` | `int32_t` |
| `vision_set_exposure(port, exposure)` | `int32_t` |
| `vision_set_led(port, rgb)` | `int32_t` |
| `vision_set_white_balance(port, rgb)` | `int32_t` |
| `vision_set_zero_point(port, zero)` | `int32_t` |
| `vision_set_wifi_mode(port, enable)` | `int32_t` |
| `vision_print_signature(sig)` | `void` |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Vision : public Device {
public:
    Vision(uint8_t port, vision_zero_e_t zero_point = E_VISION_ZERO_TOPLEFT);

    std::int32_t clear_led();
    static vision_signature_s_t signature_from_utility(uint8_t id, ...);
    vision_color_code_t create_color_code(uint32_t sig1, uint32_t sig2, ...);
    vision_object_s_t get_by_size(uint32_t size_id);
    vision_object_s_t get_by_sig(uint32_t size_id, uint32_t sig_id);
    vision_object_s_t get_by_code(uint32_t size_id, vision_color_code_t code);
    std::int32_t get_exposure();
    std::int32_t get_object_count();
    vision_signature_s_t get_signature(uint8_t sig_id);
    std::int32_t get_white_balance();
    std::int32_t read_by_size(uint32_t size_id, uint32_t count, vision_object_s_t* arr);
    std::int32_t read_by_sig(uint32_t size_id, uint32_t sig_id, uint32_t count, vision_object_s_t* arr);
    std::int32_t read_by_code(uint32_t size_id, vision_color_code_t code, uint32_t count, vision_object_s_t* arr);
    static void print_signature(vision_signature_s_t sig);
    std::int32_t set_auto_white_balance(uint8_t enable);
    std::int32_t set_exposure(uint8_t exposure);
    std::int32_t set_led(int32_t rgb);
    std::int32_t set_signature(uint8_t sig_id, vision_signature_s_t* sig);
    std::int32_t set_white_balance(int32_t rgb);
    std::int32_t set_zero_point(vision_zero_e_t zero_point);
    std::int32_t set_wifi_mode(uint8_t enable);
    static std::vector<Vision> get_vision();
};

const Vision operator"" _vis(unsigned long long int port);
} // namespace v5
} // namespace pros
```

---

## 15. AI Vision Sensor

**Headers**: `pros/ai_vision.h`, `pros/ai_vision.hpp`

### C Enums
```c
typedef enum aivision_detected_type_e {
    E_AIVISION_DETECTED_COLOR  = 1,
    E_AIVISION_DETECTED_CODE   = 2,
    E_AIVISION_DETECTED_OBJECT = 4,
    E_AIVISION_DETECTED_TAG    = 8
} aivision_detected_type_e_t;

typedef enum aivision_mode_type_e {
    E_AIVISION_MODE_TAGS        = 1,
    E_AIVISION_MODE_COLORS      = 2,
    E_AIVISION_MODE_OBJECTS     = 4,
    E_AIVISION_MODE_COLOR_MERGE = 16,
    E_AIVISION_MODE_ALL         = E_AIVISION_MODE_TAGS | E_AIVISION_MODE_COLORS | E_AIVISION_MODE_OBJECTS
} aivision_mode_type_e_t;

typedef enum aivision_tag_family_e {
    E_AIVISION_TAG_CIRCLE_21H7 = 0,
    E_AIVISION_TAG_16H5        = 1,
    E_AIVISION_TAG_25H9        = 2,
    E_AIVISION_TAG_36H11       = 3
} aivision_tag_family_e_t;
```

### C Structs
```c
typedef struct __attribute__((packed)) aivision_color_s {
    uint8_t id;
    double x_offset, y_offset, width, height;
    // internal fields: score, angle...
} aivision_color_s_t;

typedef struct __attribute__((packed)) aivision_code_s {
    uint8_t id;
    double x_offset, y_offset, width, height;
    // n colors, color descriptions
} aivision_code_s_t;

typedef struct __attribute__((packed)) aivision_object_s {
    aivision_detected_type_e_t type;
    union {
        aivision_color_s_t color;
        // tag fields
        // element fields
    };
} aivision_object_s_t;
```

### C Constants
```c
#define AIVISION_MAX_OBJECT_COUNT     24
#define AIVISION_MAX_CLASSNAME_COUNT  20
```

### C Functions
| Function | Returns |
|---|---|
| `aivision_reset(port)` | `int32_t` |
| `aivision_enable_detection_type(port, type)` | `int32_t` |
| `aivision_disable_detection_type(port, type)` | `int32_t` |
| `aivision_get_enabled_detection_types(port)` | `int32_t` |
| `aivision_set_detection_types(port, types)` | `int32_t` |
| `aivision_set_tag_family(port, family)` | `int32_t` |
| `aivision_set_color(port, id, color)` | `int32_t` |
| `aivision_get_color(port, id)` | `aivision_color_s_t` |
| `aivision_set_code(port, id, color_ids...)` | `int32_t` |
| `aivision_get_code(port, id)` | `aivision_code_s_t` |
| `aivision_get_class_name(port, id, buffer, len)` | `int32_t` |
| `aivision_set_usb_bounding_box_overlay(port, enable)` | `int32_t` |
| `aivision_start_awb(port)` | `int32_t` |
| `aivision_get_object_count(port)` | `int32_t` |
| `aivision_get_object(port, index)` | `aivision_object_s_t` |
| `aivision_get_temperature(port)` | `double` |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class AIVision : public Device {
public:
    // C++ Enums (scoped)
    enum class DetectType { color = 1, code = 2, object = 4, tag = 8 };
    enum class ModeType { tags = 1, colors = 2, objects = 4, color_merge = 16, all = 7 };
    enum class TagFamily { circle_21h7 = 0, tag_16h5 = 1, tag_25h9 = 2, tag_36h11 = 3 };

    // Type aliases
    using Color = aivision_color_s_t;
    using Code = aivision_code_s_t;
    using Object = aivision_object_s_t;

    AIVision(uint8_t port);

    static bool is_type(Object obj, DetectType type);
    std::int32_t reset();

    // Detection type management (bitmask + variadic)
    std::int32_t enable_detection_type(aivision_mode_type_e_t type);
    std::int32_t disable_detection_type(aivision_mode_type_e_t type);
    std::int32_t get_enabled_detection_types();
    std::int32_t set_detection_types(aivision_mode_type_e_t types);
    template <typename... Args>
    std::int32_t set_detection_types(ModeType type, Args... args);

    std::int32_t set_tag_family(aivision_tag_family_e_t family);
    std::int32_t set_color(uint32_t id, Color color);
    Color get_color(uint32_t id);
    std::int32_t set_code(uint32_t id, ...);
    Code get_code(uint32_t id);
    std::int32_t start_awb();
    std::optional<std::string> get_class_name(std::int32_t id);
    std::int32_t get_object_count();
    Object get_object(uint32_t index);
    std::vector<Object> get_all_objects();
};
} // namespace v5
} // namespace pros
```

---

## 16. Screen (Brain Display)

**Headers**: `pros/screen.h`, `pros/screen.hpp`

### C Enums & Types
```c
typedef enum text_format_e {
    E_TEXT_SMALL         = 0,
    E_TEXT_MEDIUM        = 1,
    E_TEXT_LARGE         = 2,
    E_TEXT_MEDIUM_CENTER = 3,
    E_TEXT_LARGE_CENTER  = 4
} text_format_e_t;

typedef enum last_touch_e {
    E_TOUCH_RELEASED = 0,
    E_TOUCH_PRESSED  = 1,
    E_TOUCH_HELD     = 2,
    E_TOUCH_ERROR    = 3  // PROS_ERR
} last_touch_e_t;

typedef struct screen_touch_status_s {
    last_touch_e_t touch_status;
    int16_t x, y;
    int32_t press_count, release_count;
} screen_touch_status_s_t;

typedef void (*touch_event_cb_fn_t)(void);
```

### C Functions — Drawing
| Function | Description |
|---|---|
| `screen_set_pen(color)` | Set pen color (uint32_t) |
| `screen_set_eraser(color)` | Set eraser color |
| `screen_get_pen()` | Get pen color |
| `screen_get_eraser()` | Get eraser color |
| `screen_erase()` | Erase entire screen |
| `screen_scroll(start_line, lines)` | Scroll screen |
| `screen_scroll_area(x0,y0,x1,y1, lines)` | Scroll area |
| `screen_copy_area(x0,y0,x1,y1, *buf, stride)` | Copy buffer to screen |
| `screen_draw_pixel(x, y)` | Draw pixel |
| `screen_erase_pixel(x, y)` | Erase pixel |
| `screen_draw_line(x0,y0,x1,y1)` | Draw line |
| `screen_erase_line(x0,y0,x1,y1)` | Erase line |
| `screen_draw_rect(x0,y0,x1,y1)` | Draw rectangle outline |
| `screen_erase_rect(x0,y0,x1,y1)` | Erase rectangle |
| `screen_fill_rect(x0,y0,x1,y1)` | Fill rectangle |
| `screen_draw_circle(x, y, r)` | Draw circle outline |
| `screen_erase_circle(x, y, r)` | Erase circle |
| `screen_fill_circle(x, y, r)` | Fill circle |

### C Functions — Text
| Function | Description |
|---|---|
| `screen_print(fmt, line, text, ...)` | Print text on line |
| `screen_print_at(fmt, x, y, text, ...)` | Print text at coords |
| `screen_vprintf(fmt, line, text, args)` | Print with va_list |
| `screen_vprintf_at(fmt, x, y, text, args)` | Print at coords with va_list |

### C Functions — Touch
| Function | Returns |
|---|---|
| `screen_touch_status()` | `screen_touch_status_s_t` |
| `screen_touch_callback(cb, event_type)` | `int32_t` |

### C++ (namespace `pros::screen`)
Same functions as C, plus template `print<Params>` overloads with variadic args. Uses same types.

---

## 17. LLEMU (Legacy LCD Emulator)

**Headers**: `pros/llemu.h`, `pros/llemu.hpp`

### C Functions
| Function | Returns |
|---|---|
| `lcd_initialize()` | `bool` |
| `lcd_shutdown()` | `bool` |
| `lcd_is_initialized()` | `bool` |
| `lcd_print(line, fmt, ...)` | `bool` |
| `lcd_set_text(line, text)` | `bool` |
| `lcd_clear()` | `bool` |
| `lcd_clear_line(line)` | `bool` |
| `lcd_register_btn0_cb(cb)` | `void` |
| `lcd_register_btn1_cb(cb)` | `void` |
| `lcd_register_btn2_cb(cb)` | `void` |
| `lcd_read_buttons()` | `uint8_t` |

### C++ (namespace `pros::lcd`)
```cpp
namespace pros::lcd {
    bool is_initialized();
    bool initialize();
    bool shutdown();
    bool set_text(std::int16_t line, std::string text);
    bool clear();
    bool clear_line(std::int16_t line);
    void register_btn0_cb(lcd_btn_cb_fn_t cb);
    void register_btn1_cb(lcd_btn_cb_fn_t cb);
    void register_btn2_cb(lcd_btn_cb_fn_t cb);
    template <typename... Params>
    bool print(std::int16_t line, const char* fmt, Params... args);
    std::uint8_t read_buttons();
}
```

### Constants
```cpp
#define LCD_BTN_LEFT   4
#define LCD_BTN_CENTER 2
#define LCD_BTN_RIGHT  1
```
Type: `typedef void (*lcd_btn_cb_fn_t)(void);`

---

## 18. VEXlink (Radio)

**Headers**: `pros/link.h`, `pros/link.hpp`

### C Enum & Constant
```c
typedef enum link_type_e {
    E_LINK_RECIEVER    = 0,  // (sic - typo in API)
    E_LINK_TRANSMITTER = 1,
    E_LINK_RX          = 0,
    E_LINK_TX          = 1
} link_type_e_t;

#define LINK_BUFFER_SIZE 512
```

### C Functions
| Function | Returns |
|---|---|
| `link_init(port, name, type)` | `int32_t` |
| `link_init_override(port, name, type)` | `int32_t` |
| `link_connected(port)` | `bool` |
| `link_raw_receivable_size(port)` | `uint32_t` |
| `link_raw_transmittable_size(port)` | `uint32_t` |
| `link_transmit_raw(port, *data, size)` | `uint32_t` |
| `link_receive_raw(port, *dest, size)` | `uint32_t` |
| `link_transmit(port, *data, size)` | `uint32_t` |
| `link_receive(port, *dest, size)` | `uint32_t` |
| `link_clear_receive_buf(port)` | `uint32_t` |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Link : public Device {
public:
    Link(uint8_t port, const char* name, link_type_e_t type, bool override = false);

    bool connected() const;
    uint32_t raw_receivable_size() const;
    uint32_t raw_transmittable_size() const;
    uint32_t transmit_raw(void* data, uint16_t size) const;
    uint32_t receive_raw(void* dest, uint16_t size) const;
    uint32_t transmit(void* data, uint16_t size) const;
    uint32_t receive(void* dest, uint16_t size) const;
    uint32_t clear_receive_buf() const;
};
} // namespace v5
} // namespace pros
```

---

## 19. Serial (Generic UART)

**Headers**: `pros/serial.h`, `pros/serial.hpp`

### C Functions
| Function | Returns | Description |
|---|---|---|
| `serial_enable(port)` | `int32_t` | Enable Generic Serial |
| `serial_set_baudrate(port, baudrate)` | `int32_t` | |
| `serial_flush(port)` | `int32_t` | Flush input/output |
| `serial_get_read_avail(port)` | `int32_t` | Bytes available to read |
| `serial_get_write_free(port)` | `int32_t` | Bytes free in write buffer |
| `serial_peek_byte(port)` | `int32_t` | Peek without consuming |
| `serial_read_byte(port)` | `int32_t` | Read next byte |
| `serial_read(port, *buf, length)` | `int32_t` | Read up to length bytes |
| `serial_write_byte(port, byte)` | `int32_t` | Write one byte |
| `serial_write(port, *buf, length)` | `int32_t` | Write up to length bytes |

### C++ Class
```cpp
namespace pros {
inline namespace v5 {
class Serial : public Device {
public:
    Serial(uint8_t port, std::int32_t baudrate);
    Serial(uint8_t port);  // default baudrate

    std::int32_t set_baudrate(std::int32_t baudrate) const;
    std::int32_t flush() const;
    std::int32_t get_read_avail() const;
    std::int32_t get_write_free() const;
    std::int32_t peek_byte() const;
    std::int32_t read_byte() const;
    std::int32_t read(std::uint8_t* buffer, std::int32_t length) const;
    std::int32_t write_byte(std::uint8_t buffer) const;
    std::int32_t write(std::uint8_t* buffer, std::int32_t length) const;
};

const Serial operator"" _ser(unsigned long long int port);
} // namespace v5
} // namespace pros
```

---

## 20. RTOS

**Headers**: `pros/rtos.h`, `pros/rtos.hpp`

### C Constants & Macros
```c
#define TASK_PRIORITY_MAX         16
#define TASK_PRIORITY_MIN          1
#define TASK_PRIORITY_DEFAULT      8
#define TASK_STACK_DEPTH_DEFAULT   0x2000  // 8192 bytes
#define TASK_STACK_DEPTH_MIN       0x200   //  512 bytes
#define TASK_NAME_MAX_LEN         32
#define TIMEOUT_MAX               0xFFFFFFFF  // block forever
```

### C Types
```c
typedef void* task_t;
typedef void (*task_fn_t)(void*);
typedef void* mutex_t;
```

### C Enums
```c
typedef enum task_state_e {
    E_TASK_STATE_RUNNING   = 0,
    E_TASK_STATE_READY     = 1,
    E_TASK_STATE_BLOCKED   = 2,
    E_TASK_STATE_SUSPENDED = 3,
    E_TASK_STATE_DELETED   = 4,
    E_TASK_STATE_INVALID   = 5
} task_state_e_t;

typedef enum notify_action_e {
    E_NOTIFY_ACTION_NONE     = 0,
    E_NOTIFY_ACTION_BITS     = 1,
    E_NOTIFY_ACTION_INCR     = 2,
    E_NOTIFY_ACTION_OWRITE   = 3,
    E_NOTIFY_ACTION_NO_OWRITE = 4
} notify_action_e_t;
```

### C Functions — Tasks
| Function | Returns |
|---|---|
| `task_create(fn, param, prio, stack, name)` | `task_t` |
| `task_remove(task)` | `void` |
| `task_get_current()` | `task_t` |
| `task_get_name(task)` | `char*` |
| `task_get_priority(task)` | `uint32_t` |
| `task_set_priority(task, prio)` | `void` |
| `task_get_state(task)` | `task_state_e_t` |
| `task_suspend(task)` | `void` |
| `task_resume(task)` | `void` |
| `task_get_count()` | `uint32_t` |
| `task_delay(ms)` | `void` |
| `task_delay_until(*prev, delta)` | `void` |
| `task_notify(task)` | `uint32_t` |
| `task_notify_ext(task, value, action, *prev)` | `uint32_t` |
| `task_notify_take(clear_on_exit, timeout)` | `uint32_t` |
| `task_notify_clear(task)` | `bool` |
| `task_join(task)` | `void` |

### C Functions — Mutexes
| Function | Returns |
|---|---|
| `mutex_create()` | `mutex_t` |
| `mutex_take(mutex, timeout)` | `bool` |
| `mutex_give(mutex)` | `bool` |
| `mutex_delete(mutex)` | `void` |

### C Functions — Timing
| Function | Returns | Unit |
|---|---|---|
| `millis()` | `uint32_t` | ms since RTOS start |
| `micros()` | `uint64_t` | µs since RTOS start |
| `delay(ms)` | `void` | alias for task_delay |

### C++ Classes

```cpp
namespace pros {
inline namespace rtos {

class Task {
public:
    // Constructors
    Task(task_fn_t function, void* parameters = nullptr,
         uint32_t prio = TASK_PRIORITY_DEFAULT,
         uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT,
         const char* name = "");

    Task(task_fn_t function, void* parameters, const char* name);

    template <class F>
    Task(F&& function, uint32_t prio = TASK_PRIORITY_DEFAULT,
         uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT,
         const char* name = "");

    template <class F>
    Task(F&& function, const char* name);

    Task(task_t task);  // wrap existing handle

    void remove();
    uint32_t get_priority() const;
    void set_priority(uint32_t prio);
    task_state_e_t get_state() const;
    void suspend();
    void resume();
    const char* get_name() const;
    uint32_t notify();
    uint32_t notify_ext(uint32_t value, notify_action_e_t action, uint32_t* prev);
    bool notify_clear();
    void join();

    // Static methods
    static uint32_t notify_take(bool clear_on_exit, uint32_t timeout);
    static void delay(uint32_t ms);
    static void delay_until(uint32_t* prev, uint32_t delta);
    static uint32_t get_count();
    static task_t get_current();

    operator task_t() const;  // implicit conversion
};

struct Clock {
    using rep = uint32_t;
    using period = std::milli;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<Clock>;
    static constexpr bool is_steady = true;
    static time_point now();
};

class Mutex {
public:
    Mutex();
    // No copy, no move

    bool take(uint32_t timeout);
    bool give();

    // BasicLockable
    void lock();
    void unlock();

    // Lockable
    bool try_lock();

    // TimedLockable
    template <class Rep, class Period>
    bool try_lock_for(const std::chrono::duration<Rep, Period>& duration);
    template <class Clock, class Duration>
    bool try_lock_until(const std::chrono::time_point<Clock, Duration>& time);

private:
    std::shared_ptr<std::remove_pointer_t<mutex_t>> mutex;
};

class RecursiveMutex {
    // Same interface as Mutex but allows recursive locking by same task
    // take(), take(timeout), give(), lock(), unlock(), try_lock()
    // try_lock_for(), try_lock_until()
};

template <typename Var>
class MutexVar {
public:
    template <class... Args>
    MutexVar(Args&&... args);

    std::optional<MutexVarLock<Var>> try_lock(uint32_t timeout = TIMEOUT_MAX);
    MutexVarLock<Var> lock();
};

template <typename Var>
class MutexVarLock {
public:
    Var& operator*();
    Var* operator->();
    // RAII - unlocks on destruction
};

// Using declarations
using pros::c::millis;
using pros::c::micros;
using pros::c::delay;

} // namespace rtos
} // namespace pros
```

---

## 21. Extended API (apix)

**Header**: `pros/apix.h`

Advanced functions not in the standard API. Used for low-level access and debugging.

### Additional RTOS Primitives

```c
typedef void* queue_t;
typedef void* sem_t;

// Task extensions
bool task_abort_delay(task_t task);
void task_notify_when_deleting(task_t target, task_t notify_task, uint32_t value, notify_action_e_t action);
task_t mutex_get_owner(mutex_t mutex);

// Semaphores
sem_t sem_create(uint32_t max_count, uint32_t init_count);
void sem_delete(sem_t sem);
sem_t sem_binary_create(void);
bool sem_wait(sem_t sem, uint32_t timeout);
bool sem_post(sem_t sem);
uint32_t sem_get_count(sem_t sem);

// Queues
queue_t queue_create(uint32_t length, uint32_t item_size);
bool queue_prepend(queue_t queue, const void* item, uint32_t timeout);
bool queue_append(queue_t queue, const void* item, uint32_t timeout);
bool queue_peek(queue_t queue, void* const buffer, uint32_t timeout);
bool queue_recv(queue_t queue, void* const buffer, uint32_t timeout);
uint32_t queue_get_waiting(const queue_t queue);
uint32_t queue_get_available(const queue_t queue);
void queue_delete(queue_t queue);
void queue_reset(queue_t queue);
```

### Device Registry
```c
int registry_bind_port(uint8_t port, v5_device_e_t device_type);
int registry_unbind_port(uint8_t port);
v5_device_e_t registry_get_bound_type(uint8_t port);
v5_device_e_t registry_get_plugged_type(uint8_t port);
```

### Startup / Banner
```c
void enable_banner(bool enabled);
#define PRE_PROS_INIT_PRIORITY 101
#define ENABLE_BANNER(enabled)  // macro for global scope usage
```

### Filesystem / Serial Control
```c
int32_t serctl(uint32_t action, void* extra_arg);
int32_t fdctl(int file, uint32_t action, void* extra_arg);

// Motor reverse (exposed in apix, not main motor API)
int32_t motor_set_reversed(int8_t port, bool reverse);
int32_t motor_is_reversed(int8_t port);

// Serial control macros
#define SERCTL_ACTIVATE      10
#define SERCTL_DEACTIVATE    11
#define SERCTL_BLKWRITE      12
#define SERCTL_NOBLKWRITE    13
#define SERCTL_ENABLE_COBS   14
#define SERCTL_DISABLE_COBS  15
#define DEVCTL_FIONREAD      16
#define DEVCTL_SET_BAUDRATE  17
#define DEVCTL_FIONWRITE     18
```

---

## 22. Colors

**Headers**: `pros/colors.h`, `pros/colors.hpp`

### C Macros
```c
#define RGB2COLOR(R, G, B)  ((R & 0xFF) << 16 | (G & 0xFF) << 8 | (B & 0xFF))
#define COLOR2R(COLOR)      ((COLOR >> 16) & 0xFF)
#define COLOR2G(COLOR)      ((COLOR >> 8)  & 0xFF)
#define COLOR2B(COLOR)      (COLOR & 0xFF)
```

### C Enum — ~140 Named Colors
```c
typedef enum color_e {
    COLOR_ALICE_BLUE        = 0x00F0F8FF,
    COLOR_ANTIQUE_WHITE     = 0x00FAEBD7,
    // ... (all standard HTML/CSS named colors)
    COLOR_RED               = 0x00FF0000,
    COLOR_GREEN             = 0x00008000,
    COLOR_BLUE              = 0x000000FF,
    COLOR_WHITE             = 0x00FFFFFF,
    COLOR_BLACK             = 0x00000000,
    // ... ~140 total entries
} color_e_t;
```

### C++ Enum Class
```cpp
namespace pros {
enum class Color : uint32_t {
    alice_blue        = 0x00F0F8FF,
    antique_white     = 0x00FAEBD7,
    // ... same ~140 colors in lowercase_snake_case
    red               = 0x00FF0000,
    green             = 0x00008000,
    blue              = 0x000000FF,
    white             = 0x00FFFFFF,
    black             = 0x00000000,
    // ...
};
} // namespace pros
```

---

## Summary Statistics

| Category | C Functions | C++ Classes | Enums | Structs | Constants |
|---|---|---|---|---|---|
| Motor | ~30 | Motor, AbstractMotor | 6 | 2 | — |
| Motor Group | — | MotorGroup | — | — | — |
| Controller/Misc | ~20 | Controller | 3 | — | 5 |
| ADI | ~25 | 12 classes | 2 | — | 6 |
| ADI Expander | ~25 | (via ADI classes) | — | — | — |
| IMU | ~25 | Imu | 2 | 3 | — |
| Rotation | ~11 | Rotation | — | — | 1 |
| Distance | ~4 | Distance | — | — | — |
| Optical | ~14 | Optical | 1 | 3 | — |
| GPS | ~25 | Gps | — | 4 | — |
| Vision | ~18 | Vision | 2 | 2 | 3 |
| AI Vision | ~16 | AIVision | 3 | 3 | 2 |
| Screen | ~20 | (namespace) | 2 | 1 | — |
| LLEMU | ~10 | (namespace) | — | — | 3 |
| Link | ~10 | Link | 1 | — | 1 |
| Serial | ~10 | Serial | — | — | — |
| RTOS | ~20 | Task, Mutex, RecursiveMutex, MutexVar, Clock | 2 | — | 6 |
| Extended API | ~20 | — | — | — | 8 |
| Device | ~3 | Device | 1 | — | — |
| Colors | — | — | 2 | — | 4 |
| **Total** | **~306** | **~20** | **~25** | **~18** | **~39** |

---

## Port Architecture Quick Reference

- **Smart Ports**: 1-21 (V5 devices: motors, sensors)
- **Internal ADI Port**: 22 (brain's built-in 3-wire ports)
- **ADI Ports**: 1-8 (or 'a'-'h' / 'A'-'H')
- **Negative port number**: Reversed motor (C++ convention)
- **Zero-indexed ports**: Only in `registry_*` functions from apix (0-20)

## Key Design Patterns for Simulator

1. **Port-based access**: All device access is via port number. The simulator needs a port registry mapping port → device state.
2. **Error via errno**: Every function that can fail sets `errno` and returns `PROS_ERR`/`PROS_ERR_F`. The simulator must replicate this.
3. **Dual API**: C functions wrap to the same internal state as C++ classes. Implement once, expose both.
4. **AbstractMotor pattern**: Motor and MotorGroup share an interface. All methods have indexed `(index=0)` and `_all()` overloads.
5. **FreeRTOS semantics**: Tasks, mutexes, semaphores, queues all follow FreeRTOS semantics (priority-based preemptive scheduling, priority inheritance on mutexes).
6. **Device auto-detection**: `registry_get_plugged_type()` vs `registry_get_bound_type()` separation.
