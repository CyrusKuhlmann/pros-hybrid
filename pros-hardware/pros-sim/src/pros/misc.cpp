/**
 * PROS Misc C API stub implementations for simulation
 * (Controller, Competition, Battery, USD)
 */
#include "pros/misc.h"

#include <cstdarg>

// Provide the baked_date and baked_time symbols
extern "C" {
namespace pros {

const char* baked_date = "Simulation";
const char* baked_time = "00:00:00";

namespace c {

// Competition
uint8_t competition_get_status(void) { return 0; }
uint8_t competition_is_disabled(void) { return 0; }
uint8_t competition_is_connected(void) { return 0; }
uint8_t competition_is_autonomous(void) { return 0; }
uint8_t competition_is_field(void) { return 0; }
uint8_t competition_is_switch(void) { return 0; }

// Controller
int32_t controller_is_connected(controller_id_e_t id) {
  (void)id;
  return 0;
}
int32_t controller_get_analog(controller_id_e_t id,
                              controller_analog_e_t channel) {
  (void)id;
  (void)channel;
  return 0;
}
int32_t controller_get_battery_capacity(controller_id_e_t id) {
  (void)id;
  return 100;
}
int32_t controller_get_battery_level(controller_id_e_t id) {
  (void)id;
  return 100;
}
int32_t controller_get_digital(controller_id_e_t id,
                               controller_digital_e_t button) {
  (void)id;
  (void)button;
  return 0;
}
int32_t controller_get_digital_new_press(controller_id_e_t id,
                                         controller_digital_e_t button) {
  (void)id;
  (void)button;
  return 0;
}
int32_t controller_get_digital_new_release(controller_id_e_t id,
                                           controller_digital_e_t button) {
  (void)id;
  (void)button;
  return 0;
}
int32_t controller_print(controller_id_e_t id, uint8_t line, uint8_t col,
                         const char* fmt, ...) {
  (void)id;
  (void)line;
  (void)col;
  (void)fmt;
  return 1;
}
int32_t controller_set_text(controller_id_e_t id, uint8_t line, uint8_t col,
                            const char* str) {
  (void)id;
  (void)line;
  (void)col;
  (void)str;
  return 1;
}
int32_t controller_clear_line(controller_id_e_t id, uint8_t line) {
  (void)id;
  (void)line;
  return 1;
}
int32_t controller_clear(controller_id_e_t id) {
  (void)id;
  return 1;
}
int32_t controller_rumble(controller_id_e_t id, const char* rumble_pattern) {
  (void)id;
  (void)rumble_pattern;
  return 1;
}

// Battery
int32_t battery_get_voltage(void) { return 12000; }
int32_t battery_get_current(void) { return 0; }
double battery_get_temperature(void) { return 25.0; }
double battery_get_capacity(void) { return 100.0; }

// SD Card
int32_t usd_is_installed(void) { return 0; }
int32_t usd_list_files(const char* path, char* buffer, int32_t len) {
  (void)path;
  (void)buffer;
  (void)len;
  return 0;
}

}  // namespace c
}  // namespace pros
}  // extern "C"
