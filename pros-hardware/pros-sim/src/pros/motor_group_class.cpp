/**
 * PROS MotorGroup C++ class stub implementations for simulation
 */
#include <cstdlib>

#include "pros/motor_group.hpp"

namespace pros {
inline namespace v5 {

MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports,
                       const MotorGears gearset, const MotorUnits encoder_units)
    : _ports(ports) {
  (void)gearset;
  (void)encoder_units;
}

MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports,
                       const MotorGears gearset, const MotorUnits encoder_units)
    : _ports(ports) {
  (void)gearset;
  (void)encoder_units;
}

MotorGroup::MotorGroup(AbstractMotor& motor) {
  auto ports = motor.get_port_all();
  _ports = ports;
}

void MotorGroup::operator+=(AbstractMotor& other) {
  auto ports = other.get_port_all();
  _ports.insert(_ports.end(), ports.begin(), ports.end());
}

void MotorGroup::append(AbstractMotor& other) { *this += other; }

void MotorGroup::erase_port(std::int8_t port) {
  for (auto it = _ports.begin(); it != _ports.end(); ++it) {
    if (*it == port) {
      _ports.erase(it);
      return;
    }
  }
}

std::int8_t MotorGroup::size() const { return (std::int8_t)_ports.size(); }
std::int8_t MotorGroup::get_port(const std::uint8_t index) const {
  return index < _ports.size() ? _ports[index] : 0;
}
std::vector<std::int8_t> MotorGroup::get_port_all() const { return _ports; }

// Movement functions
std::int32_t MotorGroup::move(std::int32_t voltage) const {
  (void)voltage;
  return 1;
}
std::int32_t MotorGroup::move_absolute(const double position,
                                       const std::int32_t velocity) const {
  (void)position;
  (void)velocity;
  return 1;
}
std::int32_t MotorGroup::move_relative(const double position,
                                       const std::int32_t velocity) const {
  (void)position;
  (void)velocity;
  return 1;
}
std::int32_t MotorGroup::move_velocity(const std::int32_t velocity) const {
  (void)velocity;
  return 1;
}
std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
  (void)voltage;
  return 1;
}
std::int32_t MotorGroup::brake() const { return 1; }
std::int32_t MotorGroup::modify_profiled_velocity(
    const std::int32_t velocity) const {
  (void)velocity;
  return 1;
}

// Telemetry - single index
double MotorGroup::get_target_position(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
std::int32_t MotorGroup::get_target_velocity(const std::uint8_t index) const {
  (void)index;
  return 0;
}
double MotorGroup::get_actual_velocity(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
std::int32_t MotorGroup::get_current_draw(const std::uint8_t index) const {
  (void)index;
  return 0;
}
std::int32_t MotorGroup::get_direction(const std::uint8_t index) const {
  (void)index;
  return 1;
}
double MotorGroup::get_efficiency(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
std::uint32_t MotorGroup::get_faults(const std::uint8_t index) const {
  (void)index;
  return 0;
}
std::uint32_t MotorGroup::get_flags(const std::uint8_t index) const {
  (void)index;
  return 0;
}
double MotorGroup::get_position(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
double MotorGroup::get_power(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
std::int32_t MotorGroup::get_raw_position(std::uint32_t* const timestamp,
                                          const std::uint8_t index) const {
  (void)timestamp;
  (void)index;
  return 0;
}
double MotorGroup::get_temperature(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
double MotorGroup::get_torque(const std::uint8_t index) const {
  (void)index;
  return 0.0;
}
std::int32_t MotorGroup::get_voltage(const std::uint8_t index) const {
  (void)index;
  return 0;
}
std::int32_t MotorGroup::is_over_current(const std::uint8_t index) const {
  (void)index;
  return 0;
}
std::int32_t MotorGroup::is_over_temp(const std::uint8_t index) const {
  (void)index;
  return 0;
}

// Telemetry - all
std::vector<double> MotorGroup::get_target_position_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<std::int32_t> MotorGroup::get_target_velocity_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}
std::vector<double> MotorGroup::get_actual_velocity_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<std::int32_t> MotorGroup::get_current_draw_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}
std::vector<std::int32_t> MotorGroup::get_direction_all() const {
  return std::vector<std::int32_t>(_ports.size(), 1);
}
std::vector<double> MotorGroup::get_efficiency_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<std::uint32_t> MotorGroup::get_faults_all() const {
  return std::vector<std::uint32_t>(_ports.size(), 0);
}
std::vector<std::uint32_t> MotorGroup::get_flags_all() const {
  return std::vector<std::uint32_t>(_ports.size(), 0);
}
std::vector<double> MotorGroup::get_position_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<double> MotorGroup::get_power_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<std::int32_t> MotorGroup::get_raw_position_all(
    std::uint32_t* const timestamp) const {
  (void)timestamp;
  return std::vector<std::int32_t>(_ports.size(), 0);
}
std::vector<double> MotorGroup::get_temperature_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<double> MotorGroup::get_torque_all() const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<std::int32_t> MotorGroup::get_voltage_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}
std::vector<std::int32_t> MotorGroup::is_over_current_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}
std::vector<std::int32_t> MotorGroup::is_over_temp_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}

// Configuration - getters
MotorBrake MotorGroup::get_brake_mode(const std::uint8_t index) const {
  (void)index;
  return MotorBrake::coast;
}
std::vector<MotorBrake> MotorGroup::get_brake_mode_all() const {
  return std::vector<MotorBrake>(_ports.size(), MotorBrake::coast);
}
std::int32_t MotorGroup::get_current_limit(const std::uint8_t index) const {
  (void)index;
  return 2500;
}
std::vector<std::int32_t> MotorGroup::get_current_limit_all() const {
  return std::vector<std::int32_t>(_ports.size(), 2500);
}
MotorUnits MotorGroup::get_encoder_units(const std::uint8_t index) const {
  (void)index;
  return MotorUnits::degrees;
}
std::vector<MotorUnits> MotorGroup::get_encoder_units_all() const {
  return std::vector<MotorUnits>(_ports.size(), MotorUnits::degrees);
}
MotorGears MotorGroup::get_gearing(const std::uint8_t index) const {
  (void)index;
  return MotorGears::green;
}
std::vector<MotorGears> MotorGroup::get_gearing_all() const {
  return std::vector<MotorGears>(_ports.size(), MotorGears::green);
}
std::int32_t MotorGroup::get_voltage_limit(const std::uint8_t index) const {
  (void)index;
  return 0;
}
std::vector<std::int32_t> MotorGroup::get_voltage_limit_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}
std::int32_t MotorGroup::is_reversed(const std::uint8_t index) const {
  (void)index;
  return 0;
}
std::vector<std::int32_t> MotorGroup::is_reversed_all() const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}
MotorType MotorGroup::get_type(const std::uint8_t index) const {
  (void)index;
  return MotorType::v5;
}
std::vector<MotorType> MotorGroup::get_type_all() const {
  return std::vector<MotorType>(_ports.size(), MotorType::v5);
}

// Configuration - setters
std::int32_t MotorGroup::set_brake_mode(const MotorBrake mode,
                                        const std::uint8_t index) const {
  (void)mode;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t mode,
                                        const std::uint8_t index) const {
  (void)mode;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake mode) const {
  (void)mode;
  return 1;
}
std::int32_t MotorGroup::set_brake_mode_all(
    const pros::motor_brake_mode_e_t mode) const {
  (void)mode;
  return 1;
}
std::int32_t MotorGroup::set_current_limit(const std::int32_t limit,
                                           const std::uint8_t index) const {
  (void)limit;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_current_limit_all(const std::int32_t limit) const {
  (void)limit;
  return 1;
}
std::int32_t MotorGroup::set_encoder_units(const MotorUnits units,
                                           const std::uint8_t index) const {
  (void)units;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_encoder_units(
    const pros::motor_encoder_units_e_t units, const std::uint8_t index) const {
  (void)units;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits units) const {
  (void)units;
  return 1;
}
std::int32_t MotorGroup::set_encoder_units_all(
    const pros::motor_encoder_units_e_t units) const {
  (void)units;
  return 1;
}
std::int32_t MotorGroup::set_gearing(const MotorGears gearset,
                                     const std::uint8_t index) const {
  (void)gearset;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_gearing(const pros::motor_gearset_e_t gearset,
                                     const std::uint8_t index) const {
  (void)gearset;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_gearing(
    const std::vector<MotorGears> gearsets) const {
  (void)gearsets;
  return 1;
}
std::int32_t MotorGroup::set_gearing(
    const std::vector<pros::motor_gearset_e_t> gearsets) const {
  (void)gearsets;
  return 1;
}
std::int32_t MotorGroup::set_gearing_all(const MotorGears gearset) const {
  (void)gearset;
  return 1;
}
std::int32_t MotorGroup::set_gearing_all(
    const pros::motor_gearset_e_t gearset) const {
  (void)gearset;
  return 1;
}
std::int32_t MotorGroup::set_reversed(const bool reverse,
                                      const std::uint8_t index) {
  (void)reverse;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_reversed_all(const bool reverse) {
  (void)reverse;
  return 1;
}
std::int32_t MotorGroup::set_voltage_limit(const std::int32_t limit,
                                           const std::uint8_t index) const {
  (void)limit;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_voltage_limit_all(const std::int32_t limit) const {
  (void)limit;
  return 1;
}
std::int32_t MotorGroup::set_zero_position(const double position,
                                           const std::uint8_t index) const {
  (void)position;
  (void)index;
  return 1;
}
std::int32_t MotorGroup::set_zero_position_all(const double position) const {
  (void)position;
  return 1;
}
std::int32_t MotorGroup::tare_position(const std::uint8_t index) const {
  (void)index;
  return 1;
}
std::int32_t MotorGroup::tare_position_all() const { return 1; }

}  // namespace v5
}  // namespace pros
