/**
 * PROS IMU C++ class stub implementations for simulation
 */
#include "pros/imu.hpp"

namespace pros {
inline namespace v5 {

Imu Imu::get_imu() { return Imu(1); }
std::vector<Imu> Imu::get_all_devices() { return {}; }
std::int32_t Imu::reset(bool blocking) const {
  (void)blocking;
  return 1;
}
std::int32_t Imu::set_data_rate(std::uint32_t rate) const {
  (void)rate;
  return 1;
}
double Imu::get_rotation() const { return 0.0; }
double Imu::get_heading() const { return 0.0; }
quaternion_s_t Imu::get_quaternion() const {
  quaternion_s_t q = {};
  q.w = 1.0;
  return q;
}
euler_s_t Imu::get_euler() const {
  euler_s_t e = {};
  return e;
}
double Imu::get_pitch() const { return 0.0; }
double Imu::get_roll() const { return 0.0; }
double Imu::get_yaw() const { return 0.0; }
imu_gyro_s_t Imu::get_gyro_rate() const {
  imu_gyro_s_t g = {};
  return g;
}
std::int32_t Imu::tare_rotation() const { return 1; }
std::int32_t Imu::tare_heading() const { return 1; }
std::int32_t Imu::tare_pitch() const { return 1; }
std::int32_t Imu::tare_yaw() const { return 1; }
std::int32_t Imu::tare_roll() const { return 1; }
std::int32_t Imu::tare() const { return 1; }
std::int32_t Imu::tare_euler() const { return 1; }
std::int32_t Imu::set_heading(double target) const {
  (void)target;
  return 1;
}
std::int32_t Imu::set_rotation(double target) const {
  (void)target;
  return 1;
}
std::int32_t Imu::set_yaw(double target) const {
  (void)target;
  return 1;
}
std::int32_t Imu::set_pitch(double target) const {
  (void)target;
  return 1;
}
std::int32_t Imu::set_roll(double target) const {
  (void)target;
  return 1;
}
std::int32_t Imu::set_euler(euler_s_t target) const {
  (void)target;
  return 1;
}
imu_accel_s_t Imu::get_accel() const {
  imu_accel_s_t a = {};
  return a;
}
ImuStatus Imu::get_status() const { return ImuStatus::ready; }
bool Imu::is_calibrating() const { return false; }
imu_orientation_e_t Imu::get_physical_orientation() const { return E_IMU_Z_UP; }

std::ostream& operator<<(std::ostream& os, const Imu& imu) {
  os << "IMU [port: " << (int)imu.get_port() << "]";
  return os;
}

namespace literals {
const Imu operator""_imu(const unsigned long long int i) {
  return Imu((std::uint8_t)i);
}
}  // namespace literals

}  // namespace v5
}  // namespace pros
