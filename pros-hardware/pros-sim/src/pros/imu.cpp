/**
 * PROS IMU C API stub implementations for simulation
 */
#include "pros/imu.h"

extern "C" {
namespace pros {
namespace c {

int32_t imu_reset(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_reset_blocking(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_set_data_rate(uint8_t port, uint32_t rate) {
  (void)port;
  (void)rate;
  return 1;
}
double imu_get_rotation(uint8_t port) {
  (void)port;
  return 0.0;
}
double imu_get_heading(uint8_t port) {
  (void)port;
  return 0.0;
}
quaternion_s_t imu_get_quaternion(uint8_t port) {
  (void)port;
  quaternion_s_t q = {};
  q.w = 1.0;
  return q;
}
euler_s_t imu_get_euler(uint8_t port) {
  (void)port;
  euler_s_t e = {};
  return e;
}
imu_gyro_s_t imu_get_gyro_rate(uint8_t port) {
  (void)port;
  imu_gyro_s_t g = {};
  return g;
}
imu_accel_s_t imu_get_accel(uint8_t port) {
  (void)port;
  imu_accel_s_t a = {};
  return a;
}
imu_status_e_t imu_get_status(uint8_t port) {
  (void)port;
  return E_IMU_STATUS_READY;
}
int32_t imu_set_euler(uint8_t port, euler_s_t target) {
  (void)port;
  (void)target;
  return 1;
}
double imu_get_pitch(uint8_t port) {
  (void)port;
  return 0.0;
}
double imu_get_roll(uint8_t port) {
  (void)port;
  return 0.0;
}
double imu_get_yaw(uint8_t port) {
  (void)port;
  return 0.0;
}
int32_t imu_tare_heading(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_tare_rotation(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_tare_pitch(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_tare_roll(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_tare_yaw(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_tare_euler(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_tare(uint8_t port) {
  (void)port;
  return 1;
}
int32_t imu_set_rotation(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}
int32_t imu_set_heading(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}
int32_t imu_set_pitch(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}
int32_t imu_set_roll(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}
int32_t imu_set_yaw(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}
imu_orientation_e_t imu_get_physical_orientation(uint8_t port) {
  (void)port;
  return E_IMU_Z_UP;
}

}  // namespace c
}  // namespace pros
}  // extern "C"
