/**
 * PROS Distance C++ class stub implementations for simulation
 */
#include "pros/distance.hpp"

namespace pros {
inline namespace v5 {

Distance::Distance(const std::uint8_t port)
    : Device(port, DeviceType::distance) {}
std::vector<Distance> Distance::get_all_devices() { return {}; }
std::int32_t Distance::get() { return 0; }
std::int32_t Distance::get_distance() { return 0; }
std::int32_t Distance::get_confidence() { return 0; }
std::int32_t Distance::get_object_size() { return 0; }
double Distance::get_object_velocity() { return 0.0; }

std::ostream& operator<<(std::ostream& os, const Distance& dist) {
  os << "Distance [port: " << (int)dist.get_port() << "]";
  return os;
}

namespace literals {
const Distance operator""_dist(const unsigned long long int d) {
  return Distance((std::uint8_t)d);
}
}  // namespace literals

}  // namespace v5
}  // namespace pros
