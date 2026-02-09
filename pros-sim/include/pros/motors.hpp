/**
 * \file pros/motors.hpp
 *
 * Contains C++ motor classes
 */

#ifndef _PROS_MOTORS_HPP_
#define _PROS_MOTORS_HPP_

#include <cstdint>

#include "pros/abstract_motor.hpp"
#include "pros/device.hpp"
#include "pros/motors.h"

namespace pros {
inline namespace v5 {

class Motor : public AbstractMotor, public Device {
 public:
  Motor(const std::int8_t port, const MotorGears gearset = MotorGears::invalid,
        const MotorUnits encoder_units = MotorUnits::invalid);

  Motor(const Device& device) : Motor(device.get_port()) {}

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
  std::int8_t _port_signed;
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_MOTORS_HPP_
