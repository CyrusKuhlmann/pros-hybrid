/**
 * PROS GPS C++ class stub implementations for simulation
 */
#include "pros/gps.hpp"

namespace pros {
  inline namespace v5 {

    std::vector<Gps> Gps::get_all_devices() { return {}; }
    Gps Gps::get_gps() { return Gps(1); }
    std::int32_t Gps::initialize_full(double xInitial, double yInitial,
      double headingInitial, double xOffset,
      double yOffset) const {
      (void)xInitial;
      (void)yInitial;
      (void)headingInitial;
      (void)xOffset;
      (void)yOffset;
      return 1;
    }
    std::int32_t Gps::set_offset(double xOffset, double yOffset) const {
      (void)xOffset;
      (void)yOffset;
      return 1;
    }
    gps_position_s_t Gps::get_offset() const {
      gps_position_s_t p = {};
      return p;
    }
    std::int32_t Gps::set_position(double xInitial, double yInitial,
      double headingInitial) const {
      (void)xInitial;
      (void)yInitial;
      (void)headingInitial;
      return 1;
    }
    std::int32_t Gps::set_data_rate(std::uint32_t rate) const {
      (void)rate;
      return 1;
    }
    double Gps::get_error() const { return 0.0; }
    gps_status_s_t Gps::get_position_and_orientation() const {
      gps_status_s_t p = {};
      return p;
    }
    gps_position_s_t Gps::get_position() const {
      gps_position_s_t p = {};
      return p;
    }
    double Gps::get_position_x() const { return 0.0; }
    double Gps::get_position_y() const { return 0.0; }
    gps_orientation_s_t Gps::get_orientation() const {
      gps_orientation_s_t o = {};
      return o;
    }
    double Gps::get_pitch() const { return 0.0; }
    double Gps::get_roll() const { return 0.0; }
    double Gps::get_yaw() const { return 0.0; }
    double Gps::get_heading() const { return 0.0; }
    double Gps::get_heading_raw() const { return 0.0; }
    gps_gyro_s_t Gps::get_gyro_rate() const {
      gps_gyro_s_t g = {};
      return g;
    }
    double Gps::get_gyro_rate_x() const { return 0.0; }
    double Gps::get_gyro_rate_y() const { return 0.0; }
    double Gps::get_gyro_rate_z() const { return 0.0; }
    gps_accel_s_t Gps::get_accel() const {
      gps_accel_s_t a = {};
      return a;
    }
    double Gps::get_accel_x() const { return 0.0; }
    double Gps::get_accel_y() const { return 0.0; }
    double Gps::get_accel_z() const { return 0.0; }

    std::ostream& operator<<(std::ostream& os, const Gps& gps) {
      os << "GPS [port: " << (int)gps.get_port() << "]";
      return os;
    }

    namespace literals {
      const Gps operator""_gps(const unsigned long long int g) {
        return Gps((std::uint8_t)g);
      }
    }  // namespace literals

  }  // namespace v5
}  // namespace pros
