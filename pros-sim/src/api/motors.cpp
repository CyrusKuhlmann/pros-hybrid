#include "pros/motors.h"

#include <algorithm>
#include <cmath>

#include "../sim/sim_device_registry.h"
#include "pros/error.h"
#include "pros/motors.hpp"

extern "C" {
namespace pros {

int32_t motor_move(int8_t port, int32_t voltage) {
  uint8_t abs_port = static_cast<uint8_t>(std::abs(port));
  int32_t actual_voltage = (port < 0) ? -voltage : voltage;
  actual_voltage = std::clamp(actual_voltage, -127, 127);

  pros_sim::SimDeviceRegistry::instance().set_motor_voltage(abs_port,
                                                            actual_voltage);
  return 1;
}

int32_t motor_move_velocity(int8_t port, int32_t velocity) {
  // Simplified: convert velocity to voltage (rough approximation)
  uint8_t abs_port = static_cast<uint8_t>(std::abs(port));
  int32_t voltage = (velocity * 127) / 200;  // Assuming 200 RPM max
  voltage = (port < 0) ? -voltage : voltage;

  pros_sim::SimDeviceRegistry::instance().set_motor_voltage(abs_port, voltage);
  return 1;
}

double motor_get_position(int8_t port) {
  uint8_t abs_port = static_cast<uint8_t>(std::abs(port));
  auto state =
      pros_sim::SimDeviceRegistry::instance().get_motor_state(abs_port);
  return (port < 0) ? -state.position_deg : state.position_deg;
}

double motor_get_actual_velocity(int8_t port) {
  uint8_t abs_port = static_cast<uint8_t>(std::abs(port));
  auto state =
      pros_sim::SimDeviceRegistry::instance().get_motor_state(abs_port);
  return (port < 0) ? -state.velocity_rpm : state.velocity_rpm;
}

int32_t motor_get_voltage(int8_t port) {
  uint8_t abs_port = static_cast<uint8_t>(std::abs(port));
  auto state =
      pros_sim::SimDeviceRegistry::instance().get_motor_state(abs_port);
  return (port < 0) ? -state.voltage : state.voltage;
}

int32_t motor_set_zero_position(int8_t port, double position) {
  (void)port;
  (void)position;
  // Not implemented in simulator
  return 1;
}

int32_t motor_tare_position(int8_t port) {
  return motor_set_zero_position(port, 0.0);
}

int32_t motor_set_brake_mode(int8_t port, motor_brake_mode_e_t mode) {
  (void)port;
  (void)mode;
  // Send command to simulator if needed
  return 1;
}

int32_t motor_set_encoder_units(int8_t port, motor_encoder_units_e_t units) {
  (void)port;
  (void)units;
  // Simulator handles units internally
  return 1;
}

int32_t motor_set_gearset(int8_t port, motor_gearset_e_t gearset) {
  (void)port;
  (void)gearset;
  // Simulator handles gearset internally
  return 1;
}

}  // namespace pros
}  // extern "C"

namespace pros {
inline namespace v5 {

Motor::Motor(const std::int8_t port, const MotorGears gearset,
             const MotorUnits encoder_units)
    : Device(static_cast<std::uint8_t>(std::abs(port)), DeviceType::motor),
      _port_signed(port) {
  if (gearset != MotorGears::invalid) {
    set_gearset(gearset);
  }
  if (encoder_units != MotorUnits::invalid) {
    set_encoder_units(encoder_units);
  }
}

std::int32_t Motor::move(std::int32_t voltage) {
  return motor_move(_port_signed, voltage);
}

std::int32_t Motor::move_velocity(std::int32_t velocity) {
  return motor_move_velocity(_port_signed, velocity);
}

double Motor::get_position() const { return motor_get_position(_port_signed); }

double Motor::get_actual_velocity() const {
  return motor_get_actual_velocity(_port_signed);
}

std::int32_t Motor::get_voltage() const {
  return motor_get_voltage(_port_signed);
}

std::int32_t Motor::tare_position() {
  return motor_tare_position(_port_signed);
}

std::int32_t Motor::set_zero_position(double position) {
  return motor_set_zero_position(_port_signed, position);
}

std::int32_t Motor::set_brake_mode(MotorBrake mode) {
  return motor_set_brake_mode(_port_signed,
                              static_cast<motor_brake_mode_e_t>(mode));
}

std::int32_t Motor::set_encoder_units(MotorUnits units) {
  return motor_set_encoder_units(_port_signed,
                                 static_cast<motor_encoder_units_e_t>(units));
}

std::int32_t Motor::set_gearset(MotorGears gearset) {
  return motor_set_gearset(_port_signed,
                           static_cast<motor_gearset_e_t>(gearset));
}

}  // namespace v5
}  // namespace pros
