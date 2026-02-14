/**
 * PROS Misc C API implementations for simulation.
 * Controller reads state from the Python VEX simulator.
 * Battery, Competition, USD remain stubbed.
 */
#include "pros/misc.h"

#include <cstdarg>

#include "sim/sim_client.h"

using namespace pros;

// Map PROS controller analog channel enum to state struct fields.
static int get_analog_from_state(const sim::ControllerState& cs,
  controller_analog_e_t channel) {
  switch (channel) {
  case E_CONTROLLER_ANALOG_LEFT_X: return cs.left_x;
  case E_CONTROLLER_ANALOG_LEFT_Y: return cs.left_y;
  case E_CONTROLLER_ANALOG_RIGHT_X: return cs.right_x;
  case E_CONTROLLER_ANALOG_RIGHT_Y: return cs.right_y;
  default: return 0;
  }
}

// Map PROS controller digital button enum to bitmask check.
static int get_digital_from_state(const sim::ControllerState& cs,
  controller_digital_e_t button) {
  // Buttons are stored as a bitmask.  The bit index corresponds to
  // (button - E_CONTROLLER_DIGITAL_L1).
  int bit = button - E_CONTROLLER_DIGITAL_L1;
  if (bit < 0 || bit > 15) return 0;
  return (cs.buttons >> bit) & 1;
}

static std::string ctrl_id(controller_id_e_t id) {
  return id == E_CONTROLLER_PARTNER ? "partner" : "master";
}

extern "C" {

  namespace pros {

    const char* baked_date = "Simulation";
    const char* baked_time = "00:00:00";

    namespace c {

      // ── Competition ─────────────────────────────────────────────────────
      uint8_t competition_get_status(void) { return 0; }
      uint8_t competition_is_disabled(void) { return 0; }
      uint8_t competition_is_connected(void) { return 0; }
      uint8_t competition_is_autonomous(void) { return 0; }
      uint8_t competition_is_field(void) { return 0; }
      uint8_t competition_is_switch(void) { return 0; }

      // ── Controller ──────────────────────────────────────────────────────
      int32_t controller_is_connected(controller_id_e_t id) {
        (void)id;
        return sim::SimClient::instance().is_connected() ? 1 : 0;
      }
      int32_t controller_get_analog(controller_id_e_t id,
        controller_analog_e_t channel) {
        auto cs = sim::SimClient::instance().get_controller_state(ctrl_id(id));
        return get_analog_from_state(cs, channel);
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
        auto cs = sim::SimClient::instance().get_controller_state(ctrl_id(id));
        return get_digital_from_state(cs, button);
      }
      int32_t controller_get_digital_new_press(controller_id_e_t id,
        controller_digital_e_t button) {
        (void)id; (void)button;
        return 0;
      }
      int32_t controller_get_digital_new_release(controller_id_e_t id,
        controller_digital_e_t button) {
        (void)id; (void)button;
        return 0;
      }
      int32_t controller_print(controller_id_e_t id, uint8_t line, uint8_t col,
        const char* fmt, ...) {
        (void)id; (void)line; (void)col; (void)fmt;
        return 1;
      }
      int32_t controller_set_text(controller_id_e_t id, uint8_t line, uint8_t col,
        const char* str) {
        (void)id; (void)line; (void)col; (void)str;
        return 1;
      }
      int32_t controller_clear_line(controller_id_e_t id, uint8_t line) {
        (void)id; (void)line;
        return 1;
      }
      int32_t controller_clear(controller_id_e_t id) {
        (void)id;
        return 1;
      }
      int32_t controller_rumble(controller_id_e_t id, const char* rumble_pattern) {
        (void)id; (void)rumble_pattern;
        return 1;
      }

      // ── Battery ─────────────────────────────────────────────────────────
      int32_t battery_get_voltage(void) { return 12000; }
      int32_t battery_get_current(void) { return 0; }
      double battery_get_temperature(void) { return 25.0; }
      double battery_get_capacity(void) { return 100.0; }

      // ── SD Card ─────────────────────────────────────────────────────────
      int32_t usd_is_installed(void) { return 0; }
      int32_t usd_list_files(const char* path, char* buffer, int32_t len) {
        (void)path; (void)buffer; (void)len;
        return 0;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
