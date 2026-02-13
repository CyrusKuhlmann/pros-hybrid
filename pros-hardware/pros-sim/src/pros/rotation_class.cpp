/**
 * PROS Rotation C++ class stub implementations for simulation
 */
#include "pros/rotation.hpp"

namespace pros {
  inline namespace v5 {

    Rotation::Rotation(const std::int8_t port)
      : Device(std::abs(port), DeviceType::rotation) {
    }
    std::vector<Rotation> Rotation::get_all_devices() { return {}; }
    std::int32_t Rotation::reset() { return 1; }
    std::int32_t Rotation::set_data_rate(std::uint32_t rate) const {
      (void)rate;
      return 1;
    }
    std::int32_t Rotation::set_position(std::int32_t position) const {
      (void)position;
      return 1;
    }
    std::int32_t Rotation::reset_position() const { return 1; }
    std::int32_t Rotation::get_position() const { return 0; }
    std::int32_t Rotation::get_velocity() const { return 0; }
    std::int32_t Rotation::get_angle() const { return 0; }
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
