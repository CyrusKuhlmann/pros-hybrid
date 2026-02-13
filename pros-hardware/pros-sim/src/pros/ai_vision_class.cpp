/**
 * PROS AIVision C++ class stub implementations for simulation
 */
#include "pros/ai_vision.hpp"

namespace pros {
inline namespace v5 {

AIVision::AIVision(const std::uint8_t port)
    : Device(port, DeviceType::aivision) {}
std::vector<AIVision> AIVision::get_all_devices() { return {}; }

bool AIVision::is_type(const Object& object, AivisionDetectType type) {
  return object.type == static_cast<uint8_t>(type);
}

std::int32_t AIVision::reset() { return 1; }
std::int32_t AIVision::get_enabled_detection_types() { return 0; }
std::int32_t AIVision::enable_detection_types(AivisionModeType types) {
  (void)types;
  return 1;
}
std::int32_t AIVision::disable_detection_types(AivisionModeType types) {
  (void)types;
  return 1;
}
std::int32_t AIVision::set_tag_family(AivisionTagFamily family, bool all_tags) {
  (void)family;
  (void)all_tags;
  return 1;
}
std::int32_t AIVision::set_color(const Color& color) {
  (void)color;
  return 1;
}
AIVision::Color AIVision::get_color(std::uint32_t id) {
  (void)id;
  Color c = {};
  return c;
}
std::uint32_t AIVision::set_code(const Code& code) {
  (void)code;
  return 1;
}
AIVision::Code AIVision::get_code(std::uint32_t id) {
  (void)id;
  Code c = {};
  return c;
}
std::int32_t AIVision::start_awb() { return 1; }
std::int32_t AIVision::get_class_name(std::int32_t id, char* name) {
  (void)id;
  if (name) name[0] = 0;
  return 1;
}
std::optional<std::string> AIVision::get_class_name(std::int32_t id) {
  (void)id;
  return std::nullopt;
}
std::int32_t AIVision::get_object_count() { return 0; }
AIVision::Object AIVision::get_object(std::uint32_t object_index) {
  (void)object_index;
  Object o = {};
  return o;
}
std::vector<AIVision::Object> AIVision::get_all_objects() { return {}; }

}  // namespace v5
}  // namespace pros
