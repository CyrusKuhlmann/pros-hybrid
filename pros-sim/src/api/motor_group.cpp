#include "pros/motor_group.hpp"

#include "pros/motors.h"

namespace pros {
inline namespace v5 {

MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports)
    : _ports(ports.begin(), ports.end()) {}

std::int32_t MotorGroup::move(std::int32_t voltage) {
  for (auto port : _ports) {
    motor_move(port, voltage);
  }
  return 1;
}

std::int32_t MotorGroup::move_velocity(std::int32_t velocity) {
  for (auto port : _ports) {
    motor_move_velocity(port, velocity);
  }
  return 1;
}

double MotorGroup::get_position() const {
  if (_ports.empty()) return 0.0;
  double sum = 0.0;
  for (auto port : _ports) {
    sum += motor_get_position(port);
  }
  return sum / static_cast<double>(_ports.size());
}

double MotorGroup::get_actual_velocity() const {
  if (_ports.empty()) return 0.0;
  double sum = 0.0;
  for (auto port : _ports) {
    sum += motor_get_actual_velocity(port);
  }
  return sum / static_cast<double>(_ports.size());
}

std::int32_t MotorGroup::get_voltage() const {
  if (_ports.empty()) return 0;
  int32_t sum = 0;
  for (auto port : _ports) {
    sum += motor_get_voltage(port);
  }
  return sum / static_cast<int32_t>(_ports.size());
}

std::int32_t MotorGroup::tare_position() {
  for (auto port : _ports) {
    motor_tare_position(port);
  }
  return 1;
}

std::int32_t MotorGroup::set_zero_position(double position) {
  for (auto port : _ports) {
    motor_set_zero_position(port, position);
  }
  return 1;
}

std::int32_t MotorGroup::set_brake_mode(MotorBrake mode) {
  for (auto port : _ports) {
    motor_set_brake_mode(port, static_cast<motor_brake_mode_e_t>(mode));
  }
  return 1;
}

std::int32_t MotorGroup::set_encoder_units(MotorUnits units) {
  for (auto port : _ports) {
    motor_set_encoder_units(port, static_cast<motor_encoder_units_e_t>(units));
  }
  return 1;
}

std::int32_t MotorGroup::set_gearset(MotorGears gearset) {
  for (auto port : _ports) {
    motor_set_gearset(port, static_cast<motor_gearset_e_t>(gearset));
  }
  return 1;
}

}  // namespace v5
}  // namespace pros
