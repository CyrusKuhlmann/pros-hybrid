/**
 * PROS Controller, Battery, Competition, USD C++ stub implementations for
 * simulation
 */
#include "pros/misc.hpp"

namespace pros {
inline namespace v5 {

Controller::Controller(controller_id_e_t id) : _id(id) {}
std::int32_t Controller::is_connected() { return 0; }
std::int32_t Controller::get_analog(controller_analog_e_t channel) {
  (void)channel;
  return 0;
}
std::int32_t Controller::get_battery_capacity() { return 100; }
std::int32_t Controller::get_battery_level() { return 100; }
std::int32_t Controller::get_digital(controller_digital_e_t button) {
  (void)button;
  return 0;
}
std::int32_t Controller::get_digital_new_press(controller_digital_e_t button) {
  (void)button;
  return 0;
}
std::int32_t Controller::get_digital_new_release(
    controller_digital_e_t button) {
  (void)button;
  return 0;
}
std::int32_t Controller::set_text(std::uint8_t line, std::uint8_t col,
                                  const char* str) {
  (void)line;
  (void)col;
  (void)str;
  return 1;
}
std::int32_t Controller::set_text(std::uint8_t line, std::uint8_t col,
                                  const std::string& str) {
  (void)line;
  (void)col;
  (void)str;
  return 1;
}
std::int32_t Controller::clear_line(std::uint8_t line) {
  (void)line;
  return 1;
}
std::int32_t Controller::clear() { return 1; }
std::int32_t Controller::rumble(const char* rumble_pattern) {
  (void)rumble_pattern;
  return 1;
}

}  // namespace v5

namespace battery {
double get_capacity() { return 100.0; }
std::int32_t get_current() { return 0; }
double get_temperature() { return 25.0; }
std::int32_t get_voltage() { return 12000; }
}  // namespace battery

namespace competition {
std::uint8_t get_status() { return 0; }
std::uint8_t is_autonomous() { return 0; }
std::uint8_t is_connected() { return 0; }
std::uint8_t is_disabled() { return 0; }
std::uint8_t is_field_control() { return 0; }
std::uint8_t is_competition_switch() { return 0; }
}  // namespace competition

namespace usd {
std::int32_t is_installed() { return 0; }
std::int32_t list_files(const char* path, char* buffer, std::int32_t len) {
  (void)path;
  (void)buffer;
  (void)len;
  return 0;
}
}  // namespace usd

}  // namespace pros
