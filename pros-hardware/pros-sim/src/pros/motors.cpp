/**
 * PROS Motors C API stub implementations for simulation
 */
#include "pros/motors.h"

extern "C" {
namespace pros {
namespace c {

int32_t motor_move(int8_t port, int32_t voltage) {
  (void)port;
  (void)voltage;
  return 1;
}
int32_t motor_brake(int8_t port) {
  (void)port;
  return 1;
}
int32_t motor_move_absolute(int8_t port, double position,
                            const int32_t velocity) {
  (void)port;
  (void)position;
  (void)velocity;
  return 1;
}
int32_t motor_move_relative(int8_t port, double position,
                            const int32_t velocity) {
  (void)port;
  (void)position;
  (void)velocity;
  return 1;
}
int32_t motor_move_velocity(int8_t port, const int32_t velocity) {
  (void)port;
  (void)velocity;
  return 1;
}
int32_t motor_move_voltage(int8_t port, const int32_t voltage) {
  (void)port;
  (void)voltage;
  return 1;
}
int32_t motor_modify_profiled_velocity(int8_t port, const int32_t velocity) {
  (void)port;
  (void)velocity;
  return 1;
}
double motor_get_target_position(int8_t port) {
  (void)port;
  return 0.0;
}
int32_t motor_get_target_velocity(int8_t port) {
  (void)port;
  return 0;
}
double motor_get_actual_velocity(int8_t port) {
  (void)port;
  return 0.0;
}
int32_t motor_get_current_draw(int8_t port) {
  (void)port;
  return 0;
}
int32_t motor_get_direction(int8_t port) {
  (void)port;
  return 1;
}
double motor_get_efficiency(int8_t port) {
  (void)port;
  return 0.0;
}
int32_t motor_is_over_current(int8_t port) {
  (void)port;
  return 0;
}
int32_t motor_is_over_temp(int8_t port) {
  (void)port;
  return 0;
}

}  // namespace c
}  // namespace pros
}  // extern "C" (temporary)

// Enums defined at pros:: level between the c:: blocks

extern "C" {
namespace pros {
namespace c {

uint32_t motor_get_faults(int8_t port) {
  (void)port;
  return 0;
}
uint32_t motor_get_flags(int8_t port) {
  (void)port;
  return 0;
}
int32_t motor_get_raw_position(int8_t port, uint32_t* const timestamp) {
  (void)port;
  (void)timestamp;
  return 0;
}
double motor_get_position(int8_t port) {
  (void)port;
  return 0.0;
}
double motor_get_power(int8_t port) {
  (void)port;
  return 0.0;
}
double motor_get_temperature(int8_t port) {
  (void)port;
  return 0.0;
}
double motor_get_torque(int8_t port) {
  (void)port;
  return 0.0;
}
int32_t motor_get_voltage(int8_t port) {
  (void)port;
  return 0;
}
int32_t motor_set_zero_position(int8_t port, const double position) {
  (void)port;
  (void)position;
  return 1;
}
int32_t motor_tare_position(int8_t port) {
  (void)port;
  return 1;
}
int32_t motor_set_brake_mode(int8_t port, const motor_brake_mode_e_t mode) {
  (void)port;
  (void)mode;
  return 1;
}
int32_t motor_set_current_limit(int8_t port, const int32_t limit) {
  (void)port;
  (void)limit;
  return 1;
}
int32_t motor_set_encoder_units(int8_t port,
                                const motor_encoder_units_e_t units) {
  (void)port;
  (void)units;
  return 1;
}
int32_t motor_set_gearing(int8_t port, const motor_gearset_e_t gearset) {
  (void)port;
  (void)gearset;
  return 1;
}
int32_t motor_set_voltage_limit(int8_t port, const int32_t limit) {
  (void)port;
  (void)limit;
  return 1;
}
motor_brake_mode_e_t motor_get_brake_mode(int8_t port) {
  (void)port;
  return E_MOTOR_BRAKE_COAST;
}
int32_t motor_get_current_limit(int8_t port) {
  (void)port;
  return 2500;
}
motor_encoder_units_e_t motor_get_encoder_units(int8_t port) {
  (void)port;
  return E_MOTOR_ENCODER_DEGREES;
}
motor_gearset_e_t motor_get_gearing(int8_t port) {
  (void)port;
  return E_MOTOR_GEARSET_18;
}
int32_t motor_get_voltage_limit(int8_t port) {
  (void)port;
  return 0;
}
motor_type_e_t motor_get_type(int8_t port) {
  (void)port;
  return E_MOTOR_TYPE_V5;
}

}  // namespace c
}  // namespace pros
}  // extern "C"
