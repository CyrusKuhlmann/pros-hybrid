/**
 * PROS Motor C++ class stub implementations for simulation
 */
#include <cstdlib>
#include <vector>

#include "pros/motors.hpp"

namespace pros {
  inline namespace v5 {

    Motor::Motor(const std::int8_t port, const MotorGears gearset,
      const MotorUnits encoder_units)
      : Device(std::abs(port), DeviceType::motor), _port(port) {
      (void)gearset;
      (void)encoder_units;
    }

    std::vector<Motor> Motor::get_all_devices() { return {}; }

    std::int8_t Motor::get_port(const std::uint8_t index) const {
      (void)index;
      return _port;
    }
    std::int8_t Motor::size() const { return 1; }

    // Movement functions
    std::int32_t Motor::move(std::int32_t voltage) const {
      (void)voltage;
      return 1;
    }
    std::int32_t Motor::move_absolute(const double position,
      const std::int32_t velocity) const {
      (void)position;
      (void)velocity;
      return 1;
    }
    std::int32_t Motor::move_relative(const double position,
      const std::int32_t velocity) const {
      (void)position;
      (void)velocity;
      return 1;
    }
    std::int32_t Motor::move_velocity(const std::int32_t velocity) const {
      (void)velocity;
      return 1;
    }
    std::int32_t Motor::move_voltage(const std::int32_t voltage) const {
      (void)voltage;
      return 1;
    }
    std::int32_t Motor::brake() const { return 1; }
    std::int32_t Motor::modify_profiled_velocity(
      const std::int32_t velocity) const {
      (void)velocity;
      return 1;
    }

    // Telemetry functions
    double Motor::get_target_position(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_target_position_all() const { return { 0.0 }; }
    std::int32_t Motor::get_target_velocity(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::get_target_velocity_all() const { return { 0 }; }
    double Motor::get_actual_velocity(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_actual_velocity_all() const { return { 0.0 }; }
    std::int32_t Motor::get_current_draw(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::get_current_draw_all() const { return { 0 }; }
    std::int32_t Motor::get_direction(const std::uint8_t index) const {
      (void)index;
      return 1;
    }
    std::vector<std::int32_t> Motor::get_direction_all() const { return { 1 }; }
    double Motor::get_efficiency(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_efficiency_all() const { return { 0.0 }; }
    std::uint32_t Motor::get_faults(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::uint32_t> Motor::get_faults_all() const { return { 0 }; }
    std::uint32_t Motor::get_flags(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::uint32_t> Motor::get_flags_all() const { return { 0 }; }
    double Motor::get_position(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_position_all() const { return { 0.0 }; }
    double Motor::get_power(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_power_all() const { return { 0.0 }; }
    std::int32_t Motor::get_raw_position(std::uint32_t* const timestamp,
      const std::uint8_t index) const {
      (void)timestamp;
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::get_raw_position_all(
      std::uint32_t* const timestamp) const {
      (void)timestamp;
      return { 0 };
    }
    double Motor::get_temperature(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_temperature_all() const { return { 0.0 }; }
    double Motor::get_torque(const std::uint8_t index) const {
      (void)index;
      return 0.0;
    }
    std::vector<double> Motor::get_torque_all() const { return { 0.0 }; }
    std::int32_t Motor::get_voltage(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::get_voltage_all() const { return { 0 }; }
    std::int32_t Motor::is_over_current(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::is_over_current_all() const { return { 0 }; }
    std::int32_t Motor::is_over_temp(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::is_over_temp_all() const { return { 0 }; }

    // Configuration functions
    MotorBrake Motor::get_brake_mode(const std::uint8_t index) const {
      (void)index;
      return MotorBrake::coast;
    }
    std::vector<MotorBrake> Motor::get_brake_mode_all() const {
      return { MotorBrake::coast };
    }
    std::int32_t Motor::get_current_limit(const std::uint8_t index) const {
      (void)index;
      return 2500;
    }
    std::vector<std::int32_t> Motor::get_current_limit_all() const {
      return { 2500 };
    }
    MotorUnits Motor::get_encoder_units(const std::uint8_t index) const {
      (void)index;
      return MotorUnits::degrees;
    }
    std::vector<MotorUnits> Motor::get_encoder_units_all() const {
      return { MotorUnits::degrees };
    }
    MotorGears Motor::get_gearing(const std::uint8_t index) const {
      (void)index;
      return MotorGears::green;
    }
    std::vector<MotorGears> Motor::get_gearing_all() const {
      return { MotorGears::green };
    }
    std::vector<std::int8_t> Motor::get_port_all() const { return { _port }; }
    std::int32_t Motor::get_voltage_limit(const std::uint8_t index) const {
      (void)index;
      return 0;
    }
    std::vector<std::int32_t> Motor::get_voltage_limit_all() const { return { 0 }; }
    std::int32_t Motor::is_reversed(const std::uint8_t index) const {
      (void)index;
      return _port < 0 ? 1 : 0;
    }
    std::vector<std::int32_t> Motor::is_reversed_all() const {
      return { _port < 0 ? 1 : 0 };
    }
    MotorType Motor::get_type(const std::uint8_t index) const {
      (void)index;
      return MotorType::v5;
    }
    std::vector<MotorType> Motor::get_type_all() const { return { MotorType::v5 }; }

    std::int32_t Motor::set_brake_mode(const MotorBrake mode,
      const std::uint8_t index) const {
      (void)mode;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_brake_mode(const pros::motor_brake_mode_e_t mode,
      const std::uint8_t index) const {
      (void)mode;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_brake_mode_all(const MotorBrake mode) const {
      (void)mode;
      return 1;
    }
    std::int32_t Motor::set_brake_mode_all(
      const pros::motor_brake_mode_e_t mode) const {
      (void)mode;
      return 1;
    }
    std::int32_t Motor::set_current_limit(const std::int32_t limit,
      const std::uint8_t index) const {
      (void)limit;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_current_limit_all(const std::int32_t limit) const {
      (void)limit;
      return 1;
    }
    std::int32_t Motor::set_encoder_units(const MotorUnits units,
      const std::uint8_t index) const {
      (void)units;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_encoder_units(const pros::motor_encoder_units_e_t units,
      const std::uint8_t index) const {
      (void)units;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_encoder_units_all(const MotorUnits units) const {
      (void)units;
      return 1;
    }
    std::int32_t Motor::set_encoder_units_all(
      const pros::motor_encoder_units_e_t units) const {
      (void)units;
      return 1;
    }
    std::int32_t Motor::set_gearing(const MotorGears gearset,
      const std::uint8_t index) const {
      (void)gearset;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_gearing(const pros::motor_gearset_e_t gearset,
      const std::uint8_t index) const {
      (void)gearset;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_gearing_all(const MotorGears gearset) const {
      (void)gearset;
      return 1;
    }
    std::int32_t Motor::set_gearing_all(
      const pros::motor_gearset_e_t gearset) const {
      (void)gearset;
      return 1;
    }
    std::int32_t Motor::set_reversed(const bool reverse, const std::uint8_t index) {
      (void)reverse;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_reversed_all(const bool reverse) {
      (void)reverse;
      return 1;
    }
    std::int32_t Motor::set_voltage_limit(const std::int32_t limit,
      const std::uint8_t index) const {
      (void)limit;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_voltage_limit_all(const std::int32_t limit) const {
      (void)limit;
      return 1;
    }
    std::int32_t Motor::set_zero_position(const double position,
      const std::uint8_t index) const {
      (void)position;
      (void)index;
      return 1;
    }
    std::int32_t Motor::set_zero_position_all(const double position) const {
      (void)position;
      return 1;
    }
    std::int32_t Motor::tare_position(const std::uint8_t index) const {
      (void)index;
      return 1;
    }
    std::int32_t Motor::tare_position_all() const { return 1; }

    std::ostream& operator<<(std::ostream& os, const Motor& motor) {
      os << "Motor [port: " << (int)motor.get_port() << "]";
      return os;
    }

    namespace literals {
      const Motor operator""_mtr(const unsigned long long int m) {
        return Motor((std::int8_t)m);
      }
      const Motor operator""_rmtr(const unsigned long long int m) {
        return Motor(-(std::int8_t)m);
      }
    }  // namespace literals

  }  // namespace v5
}  // namespace pros
