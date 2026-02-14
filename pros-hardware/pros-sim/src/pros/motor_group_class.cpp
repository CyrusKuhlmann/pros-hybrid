/**
 * PROS MotorGroup C++ class implementations for simulation.
 * Delegates to SimClient for each port in the group.
 */
#include <cstdlib>

#include "pros/motor_group.hpp"
#include "sim/sim_client.h"

static int ap(std::int8_t port) { return std::abs(port); }
static int sg(std::int8_t port) { return port < 0 ? -1 : 1; }
static std::string bm_str(pros::MotorBrake m) {
  switch (m) {
  case pros::MotorBrake::hold: return "hold";
  case pros::MotorBrake::brake: return "brake";
  default: return "coast";
  }
}

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

    // ── Movement functions ───────────────────────────────────────────────

    std::int32_t MotorGroup::move(std::int32_t voltage) const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_move(ap(p), sg(p) * voltage);
      return 1;
    }
    std::int32_t MotorGroup::move_absolute(const double position,
      const std::int32_t velocity) const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_move_absolute(
          ap(p), sg(p) * position, velocity);
      return 1;
    }
    std::int32_t MotorGroup::move_relative(const double position,
      const std::int32_t velocity) const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_move_relative(
          ap(p), sg(p) * position, velocity);
      return 1;
    }
    std::int32_t MotorGroup::move_velocity(const std::int32_t velocity) const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_move_velocity(
          ap(p), sg(p) * velocity);
      return 1;
    }
    std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
      int v127 = static_cast<int>(
        static_cast<double>(voltage) * 127.0 / 12000.0);
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_move(ap(p), sg(p) * v127);
      return 1;
    }
    std::int32_t MotorGroup::brake() const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_brake(ap(p));
      return 1;
    }
    std::int32_t MotorGroup::modify_profiled_velocity(
      const std::int32_t velocity) const {
      (void)velocity;
      return 1;
    }

    // ── Telemetry - single index ─────────────────────────────────────────

    double MotorGroup::get_target_position(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0.0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return sg(_ports[index]) * st.target_position_deg;
    }
    std::int32_t MotorGroup::get_target_velocity(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return sg(_ports[index]) * st.target_velocity_rpm;
    }
    double MotorGroup::get_actual_velocity(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0.0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return sg(_ports[index]) * st.velocity_rpm;
    }
    std::int32_t MotorGroup::get_current_draw(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return st.current_ma;
    }
    std::int32_t MotorGroup::get_direction(const std::uint8_t index) const {
      if (index >= _ports.size()) return 1;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return (sg(_ports[index]) * st.velocity_rpm) >= 0 ? 1 : -1;
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
      if (index >= _ports.size()) return 0.0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return sg(_ports[index]) * st.position_deg;
    }
    double MotorGroup::get_power(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0.0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return (std::abs(st.voltage) / 127.0 * 12.0) *
        (st.current_ma / 1000.0);
    }
    std::int32_t MotorGroup::get_raw_position(std::uint32_t* const timestamp,
      const std::uint8_t index) const {
      if (timestamp)
        *timestamp = sim::SimClient::instance().get_timestamp_ms();
      if (index >= _ports.size()) return 0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return static_cast<int32_t>(sg(_ports[index]) * st.position_deg * 100.0);
    }
    double MotorGroup::get_temperature(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0.0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return st.temp_c;
    }
    double MotorGroup::get_torque(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0.0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return st.torque_nm;
    }
    std::int32_t MotorGroup::get_voltage(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return sg(_ports[index]) *
        static_cast<int32_t>(st.voltage * 12000.0 / 127.0);
    }
    std::int32_t MotorGroup::is_over_current(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return st.current_ma > 2500 ? 1 : 0;
    }
    std::int32_t MotorGroup::is_over_temp(const std::uint8_t index) const {
      if (index >= _ports.size()) return 0;
      auto st = sim::SimClient::instance().get_motor_state(ap(_ports[index]));
      return st.temp_c > 55.0 ? 1 : 0;
    }

    // ── Telemetry - all ──────────────────────────────────────────────────

#define MG_ALL(T, fn) \
    std::vector<T> v; v.reserve(_ports.size()); \
    for (std::uint8_t i = 0; i < _ports.size(); ++i) v.push_back(fn(i)); \
    return v;

    std::vector<double> MotorGroup::get_target_position_all() const {
      MG_ALL(double, get_target_position)
    }
    std::vector<std::int32_t> MotorGroup::get_target_velocity_all() const {
      MG_ALL(std::int32_t, get_target_velocity)
    }
    std::vector<double> MotorGroup::get_actual_velocity_all() const {
      MG_ALL(double, get_actual_velocity)
    }
    std::vector<std::int32_t> MotorGroup::get_current_draw_all() const {
      MG_ALL(std::int32_t, get_current_draw)
    }
    std::vector<std::int32_t> MotorGroup::get_direction_all() const {
      MG_ALL(std::int32_t, get_direction)
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
      MG_ALL(double, get_position)
    }
    std::vector<double> MotorGroup::get_power_all() const {
      MG_ALL(double, get_power)
    }
    std::vector<std::int32_t> MotorGroup::get_raw_position_all(
      std::uint32_t* const timestamp) const {
      std::vector<std::int32_t> v;
      v.reserve(_ports.size());
      for (std::uint8_t i = 0; i < _ports.size(); ++i)
        v.push_back(get_raw_position(timestamp, i));
      return v;
    }
    std::vector<double> MotorGroup::get_temperature_all() const {
      MG_ALL(double, get_temperature)
    }
    std::vector<double> MotorGroup::get_torque_all() const {
      MG_ALL(double, get_torque)
    }
    std::vector<std::int32_t> MotorGroup::get_voltage_all() const {
      MG_ALL(std::int32_t, get_voltage)
    }
    std::vector<std::int32_t> MotorGroup::is_over_current_all() const {
      MG_ALL(std::int32_t, is_over_current)
    }
    std::vector<std::int32_t> MotorGroup::is_over_temp_all() const {
      MG_ALL(std::int32_t, is_over_temp)
    }

#undef MG_ALL

    // ── Configuration - getters ──────────────────────────────────────────

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
      if (index >= _ports.size()) return 0;
      return _ports[index] < 0 ? 1 : 0;
    }
    std::vector<std::int32_t> MotorGroup::is_reversed_all() const {
      std::vector<std::int32_t> v;
      for (auto p : _ports) v.push_back(p < 0 ? 1 : 0);
      return v;
    }
    MotorType MotorGroup::get_type(const std::uint8_t index) const {
      (void)index;
      return MotorType::v5;
    }
    std::vector<MotorType> MotorGroup::get_type_all() const {
      return std::vector<MotorType>(_ports.size(), MotorType::v5);
    }

    // ── Configuration - setters ──────────────────────────────────────────

    std::int32_t MotorGroup::set_brake_mode(const MotorBrake mode,
      const std::uint8_t index) const {
      if (index < _ports.size())
        sim::SimClient::instance().send_motor_set_brake_mode(
          ap(_ports[index]), bm_str(mode));
      return 1;
    }
    std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t mode,
      const std::uint8_t index) const {
      (void)index;
      std::string m;
      switch (mode) {
      case E_MOTOR_BRAKE_HOLD: m = "hold"; break;
      case E_MOTOR_BRAKE_BRAKE: m = "brake"; break;
      default: m = "coast"; break;
      }
      if (index < _ports.size())
        sim::SimClient::instance().send_motor_set_brake_mode(
          ap(_ports[index]), m);
      return 1;
    }
    std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake mode) const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_set_brake_mode(
          ap(p), bm_str(mode));
      return 1;
    }
    std::int32_t MotorGroup::set_brake_mode_all(
      const pros::motor_brake_mode_e_t mode) const {
      std::string m;
      switch (mode) {
      case E_MOTOR_BRAKE_HOLD: m = "hold"; break;
      case E_MOTOR_BRAKE_BRAKE: m = "brake"; break;
      default: m = "coast"; break;
      }
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_set_brake_mode(ap(p), m);
      return 1;
    }
    std::int32_t MotorGroup::set_current_limit(const std::int32_t limit,
      const std::uint8_t index) const {
      (void)limit; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_current_limit_all(const std::int32_t limit) const {
      (void)limit; return 1;
    }
    std::int32_t MotorGroup::set_encoder_units(const MotorUnits units,
      const std::uint8_t index) const {
      (void)units; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_encoder_units(
      const pros::motor_encoder_units_e_t units, const std::uint8_t index) const {
      (void)units; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits units) const {
      (void)units; return 1;
    }
    std::int32_t MotorGroup::set_encoder_units_all(
      const pros::motor_encoder_units_e_t units) const {
      (void)units; return 1;
    }
    std::int32_t MotorGroup::set_gearing(const MotorGears gearset,
      const std::uint8_t index) const {
      (void)gearset; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_gearing(const pros::motor_gearset_e_t gearset,
      const std::uint8_t index) const {
      (void)gearset; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_gearing(
      const std::vector<MotorGears> gearsets) const {
      (void)gearsets; return 1;
    }
    std::int32_t MotorGroup::set_gearing(
      const std::vector<pros::motor_gearset_e_t> gearsets) const {
      (void)gearsets; return 1;
    }
    std::int32_t MotorGroup::set_gearing_all(const MotorGears gearset) const {
      (void)gearset; return 1;
    }
    std::int32_t MotorGroup::set_gearing_all(
      const pros::motor_gearset_e_t gearset) const {
      (void)gearset; return 1;
    }
    std::int32_t MotorGroup::set_reversed(const bool reverse,
      const std::uint8_t index) {
      (void)reverse; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_reversed_all(const bool reverse) {
      (void)reverse; return 1;
    }
    std::int32_t MotorGroup::set_voltage_limit(const std::int32_t limit,
      const std::uint8_t index) const {
      (void)limit; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_voltage_limit_all(const std::int32_t limit) const {
      (void)limit; return 1;
    }
    std::int32_t MotorGroup::set_zero_position(const double position,
      const std::uint8_t index) const {
      (void)position; (void)index; return 1;
    }
    std::int32_t MotorGroup::set_zero_position_all(const double position) const {
      (void)position; return 1;
    }
    std::int32_t MotorGroup::tare_position(const std::uint8_t index) const {
      if (index < _ports.size())
        sim::SimClient::instance().send_motor_tare_position(ap(_ports[index]));
      return 1;
    }
    std::int32_t MotorGroup::tare_position_all() const {
      for (auto p : _ports)
        sim::SimClient::instance().send_motor_tare_position(ap(p));
      return 1;
    }

  }  // namespace v5
}  // namespace pros
