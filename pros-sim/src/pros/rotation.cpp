/**
 * PROS Rotation C API implementations for simulation.
 * Reads state from the Python VEX simulator.
 */
#include "pros/rotation.h"

#include "sim/sim_client.h"

extern "C" {
  namespace pros {
    namespace c {

      int32_t rotation_reset(uint8_t port) {
        sim::SimClient::instance().send_rotation_reset(port);
        return 1;
      }
      int32_t rotation_set_data_rate(uint8_t port, uint32_t rate) {
        (void)port; (void)rate;
        return 1;
      }
      int32_t rotation_set_position(uint8_t port, int32_t position) {
        sim::SimClient::instance().send_rotation_set_position(port, position);
        return 1;
      }
      int32_t rotation_reset_position(uint8_t port) {
        sim::SimClient::instance().send_rotation_reset(port);
        return 1;
      }
      int32_t rotation_get_position(uint8_t port) {
        return sim::SimClient::instance().get_rotation_state(port).position_cdeg;
      }
      int32_t rotation_get_velocity(uint8_t port) {
        return static_cast<int32_t>(
          sim::SimClient::instance().get_rotation_state(port).velocity_dps);
      }
      int32_t rotation_get_angle(uint8_t port) {
        int cdeg = sim::SimClient::instance()
          .get_rotation_state(port).position_cdeg;
        // Angle is 0-36000 centidegrees
        int angle = cdeg % 36000;
        if (angle < 0) angle += 36000;
        return angle;
      }
      int32_t rotation_set_reversed(uint8_t port, bool value) {
        (void)port; (void)value;
        return 1;
      }
      int32_t rotation_reverse(uint8_t port) {
        (void)port;
        return 1;
      }
      int32_t rotation_init_reverse(uint8_t port, bool reverse_flag) {
        (void)port; (void)reverse_flag;
        return 1;
      }
      int32_t rotation_get_reversed(uint8_t port) {
        (void)port;
        return 0;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
