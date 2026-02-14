/**
 * PROS Rotation C++ class implementations for simulation.
 * Reads state from the Python VEX simulator.
 */
#include "pros/rotation.hpp"
#include "sim/sim_client.h"

namespace pros {
  inline namespace v5 {

    Rotation::Rotation(const std::int8_t port)
      : Device(std::abs(port), DeviceType::rotation) {
    }
    std::vector<Rotation> Rotation::get_all_devices() { return {}; }

    std::int32_t Rotation::reset() {
      sim::SimClient::instance().send_rotation_reset(_port);
      return 1;
    }
    std::int32_t Rotation::set_data_rate(std::uint32_t rate) const {
      (void)rate;
      return 1;
    }
    std::int32_t Rotation::set_position(std::int32_t position) const {
      sim::SimClient::instance().send_rotation_set_position(_port, position);
      return 1;
    }
    std::int32_t Rotation::reset_position() const {
      sim::SimClient::instance().send_rotation_reset(_port);
      return 1;
    }
    std::int32_t Rotation::get_position() const {
      return sim::SimClient::instance().get_rotation_state(_port).position_cdeg;
    }
    std::int32_t Rotation::get_velocity() const {
      return static_cast<std::int32_t>(
        sim::SimClient::instance().get_rotation_state(_port).velocity_dps);
    }
    std::int32_t Rotation::get_angle() const {
      int cdeg = sim::SimClient::instance()
        .get_rotation_state(_port).position_cdeg;
      int angle = cdeg % 36000;
      if (angle < 0) angle += 36000;
      return angle;
    }
    std::int32_t Rotation::set_reversed(bool value) const {
      (void)value;
      return 1;
    }
    std::int32_t Rotation::reverse() const { return 1; }
    std::int32_t Rotation::get_reversed() const { return 0; }

    std::ostream& operator<<(std::ostream& os, const Rotation& rot) {
      os << "Rotation [port: " << (int)rot.get_port() << "]";
      return os;
    }

    namespace literals {
      const Rotation operator""_rot(const unsigned long long int r) {
        return Rotation((std::int8_t)r);
      }
    }  // namespace literals

  }  // namespace v5
}  // namespace pros
