/**
 * \file pros/abstract_motor.hpp
 *
 * Abstract base class for Motor and MotorGroup
 */

#ifndef _PROS_ABSTRACT_MOTOR_HPP_
#define _PROS_ABSTRACT_MOTOR_HPP_

#include <cstdint>

#include "pros/motors.h"

namespace pros {
inline namespace v5 {

enum class MotorBrake { coast = 0, brake = 1, hold = 2, invalid = INT32_MAX };

enum class MotorEncoderUnits {
  degrees = 0,
  deg = 0,
  rotations = 1,
  counts = 2,
  invalid = INT32_MAX
};

using MotorUnits = MotorEncoderUnits;

enum class MotorGears {
  ratio_36_to_1 = 0,
  red = ratio_36_to_1,
  rpm_100 = ratio_36_to_1,
  ratio_18_to_1 = 1,
  green = ratio_18_to_1,
  rpm_200 = ratio_18_to_1,
  ratio_6_to_1 = 2,
  blue = ratio_6_to_1,
  rpm_600 = ratio_6_to_1,
  invalid = INT32_MAX
};

using MotorGearset = MotorGears;
using MotorCart = MotorGears;
using MotorCartridge = MotorGears;
using MotorGear = MotorGears;

enum class MotorType { v5 = 0, exp = 1, invalid = INT32_MAX };

class AbstractMotor {
 public:
  virtual ~AbstractMotor() = default;

  virtual std::int32_t move(std::int32_t voltage) = 0;
  virtual std::int32_t move_velocity(std::int32_t velocity) = 0;
  virtual double get_position() const = 0;
  virtual double get_actual_velocity() const = 0;
  virtual std::int32_t get_voltage() const = 0;
  virtual std::int32_t tare_position() = 0;
  virtual std::int32_t set_zero_position(double position) = 0;
  virtual std::int32_t set_brake_mode(MotorBrake mode) = 0;
  virtual std::int32_t set_encoder_units(MotorUnits units) = 0;
  virtual std::int32_t set_gearset(MotorGears gearset) = 0;
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_ABSTRACT_MOTOR_HPP_
