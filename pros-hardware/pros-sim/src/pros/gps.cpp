/**
 * PROS GPS C API stub implementations for simulation
 */
#include "pros/gps.h"

extern "C" {
  namespace pros {
    namespace c {

      int32_t gps_initialize_full(uint8_t port, double xInitial, double yInitial,
        double headingInitial, double xOffset,
        double yOffset) {
        (void)port;
        (void)xInitial;
        (void)yInitial;
        (void)headingInitial;
        (void)xOffset;
        (void)yOffset;
        return 1;
      }
      int32_t gps_set_offset(uint8_t port, double xOffset, double yOffset) {
        (void)port;
        (void)xOffset;
        (void)yOffset;
        return 1;
      }
      gps_position_s_t gps_get_offset(uint8_t port) {
        (void)port;
        gps_position_s_t p = {};
        return p;
      }
      int32_t gps_set_position(uint8_t port, double xInitial, double yInitial,
        double headingInitial) {
        (void)port;
        (void)xInitial;
        (void)yInitial;
        (void)headingInitial;
        return 1;
      }
      int32_t gps_set_data_rate(uint8_t port, uint32_t rate) {
        (void)port;
        (void)rate;
        return 1;
      }
      double gps_get_error(uint8_t port) {
        (void)port;
        return 0.0;
      }
      gps_status_s_t gps_get_position_and_orientation(uint8_t port) {
        (void)port;
        gps_status_s_t s = {};
        return s;
      }
      gps_position_s_t gps_get_position(uint8_t port) {
        (void)port;
        gps_position_s_t p = {};
        return p;
      }
      double gps_get_position_x(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_position_y(uint8_t port) {
        (void)port;
        return 0.0;
      }
      gps_orientation_s_t gps_get_orientation(uint8_t port) {
        (void)port;
        gps_orientation_s_t o = {};
        return o;
      }
      double gps_get_pitch(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_roll(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_yaw(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_heading(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_heading_raw(uint8_t port) {
        (void)port;
        return 0.0;
      }
      gps_gyro_s_t gps_get_gyro_rate(uint8_t port) {
        (void)port;
        gps_gyro_s_t g = {};
        return g;
      }
      double gps_get_gyro_rate_x(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_gyro_rate_y(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_gyro_rate_z(uint8_t port) {
        (void)port;
        return 0.0;
      }
      gps_accel_s_t gps_get_accel(uint8_t port) {
        (void)port;
        gps_accel_s_t a = {};
        return a;
      }
      double gps_get_accel_x(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_accel_y(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double gps_get_accel_z(uint8_t port) {
        (void)port;
        return 0.0;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
