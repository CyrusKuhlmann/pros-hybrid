/**
 * PROS Motors C API implementations for simulation.
 * Sends commands to and reads state from the Python VEX simulator.
 */
#include "pros/motors.h"

#include <cstdlib>

#include "sim/sim_client.h"

using namespace pros;

// Helper: resolve absolute port and sign multiplier for reversed motors.
static int abs_port(int8_t port) { return std::abs(port); }
static int sign(int8_t port) { return port < 0 ? -1 : 1; }
static std::string brake_mode_str(motor_brake_mode_e_t mode) {
  switch (mode) {
    case E_MOTOR_BRAKE_HOLD:
      return "hold";
    case E_MOTOR_BRAKE_BRAKE:
      return "brake";
    default:
      return "coast";
  }
}

extern "C" {
namespace pros {
namespace c {

int32_t motor_move(int8_t port, int32_t voltage) {
  sim::SimClient::instance().send_motor_move(abs_port(port),
                                             sign(port) * voltage);
  return 1;
}
int32_t motor_brake(int8_t port) {
  sim::SimClient::instance().send_motor_move(abs_port(port), 0);
  return 1;
}
int32_t motor_move_absolute(int8_t port, double position,
                            const int32_t velocity) {
  // High-level PID belongs in user code; no-op stub.
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
  // Map velocity RPM to proportional voltage (-127..127)
  int v = static_cast<int>(static_cast<double>(velocity) * 127.0 / 200.0);
  sim::SimClient::instance().send_motor_move(abs_port(port), sign(port) * v);
  return 1;
}
int32_t motor_move_voltage(int8_t port, const int32_t voltage) {
  // PROS voltage is in mV (-12000..12000), convert to -127..127 range
  int v127 = static_cast<int>(static_cast<double>(voltage) * 127.0 / 12000.0);
  sim::SimClient::instance().send_motor_move(abs_port(port), sign(port) * v127);
  return 1;
}
int32_t motor_modify_profiled_velocity(int8_t port, const int32_t velocity) {
  (void)port;
  (void)velocity;
  return 1;
}
double motor_get_target_position(int8_t port) {
  (void)port;
  return 0.0;  // No server-side target; implement in client PID
}
int32_t motor_get_target_velocity(int8_t port) {
  (void)port;
  return 0;  // No server-side target; implement in client PID
}
double motor_get_actual_velocity(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return sign(port) * st.velocity_rpm;
}
int32_t motor_get_current_draw(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return st.current_ma;
}
int32_t motor_get_direction(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  double vel = sign(port) * st.velocity_rpm;
  return vel >= 0 ? 1 : -1;
}
double motor_get_efficiency(int8_t port) {
  (void)port;
  return 0.0;
}
int32_t motor_is_over_current(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return st.current_ma > 2500 ? 1 : 0;
}
int32_t motor_is_over_temp(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return st.temp_c > 55.0 ? 1 : 0;
}

}  // namespace c
}  // namespace pros
}  // extern "C"

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
  if (timestamp) *timestamp = sim::SimClient::instance().get_timestamp_ms();
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return static_cast<int32_t>(sign(port) * st.position_deg * 100.0);
}
double motor_get_position(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return sign(port) * st.position_deg;
}
double motor_get_power(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return (std::abs(st.voltage) / 127.0 * 12.0) * (st.current_ma / 1000.0);
}
double motor_get_temperature(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return st.temp_c;
}
double motor_get_torque(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return st.torque_nm;
}
int32_t motor_get_voltage(int8_t port) {
  auto st = sim::SimClient::instance().get_motor_state(abs_port(port));
  return sign(port) * static_cast<int32_t>(st.voltage * 12000.0 / 127.0);
}
int32_t motor_set_zero_position(int8_t port, const double position) {
  (void)port;
  (void)position;
  return 1;
}
int32_t motor_tare_position(int8_t port) {
  sim::SimClient::instance().send_motor_tare_position(abs_port(port));
  return 1;
}
int32_t motor_set_brake_mode(int8_t port, const motor_brake_mode_e_t mode) {
  sim::SimClient::instance().send_motor_set_brake_mode(abs_port(port),
                                                       brake_mode_str(mode));
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
