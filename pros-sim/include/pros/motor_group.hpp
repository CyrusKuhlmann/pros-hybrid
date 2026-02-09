/**
 * \file pros/motor_group.hpp
 *
 * Contains MotorGroup class
 */

#ifndef _PROS_MOTOR_GROUP_HPP_
#define _PROS_MOTOR_GROUP_HPP_

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "pros/abstract_motor.hpp"

namespace pros {
inline namespace v5 {

class MotorGroup : public AbstractMotor {
 public:
  MotorGroup(const std::initializer_list<std::int8_t> ports);

  // Movement functions
  std::int32_t move(std::int32_t voltage) override;
  std::int32_t move_velocity(std::int32_t velocity) override;

  // Telemetry functions
  double get_position() const override;
  double get_actual_velocity() const override;
  std::int32_t get_voltage() const override;

  // Configuration functions
  std::int32_t tare_position() override;
  std::int32_t set_zero_position(double position) override;
  std::int32_t set_brake_mode(MotorBrake mode) override;
  std::int32_t set_encoder_units(MotorUnits units) override;
  std::int32_t set_gearset(MotorGears gearset) override;

 private:
  std::vector<std::int8_t> _ports;
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_MOTOR_GROUP_HPP_
