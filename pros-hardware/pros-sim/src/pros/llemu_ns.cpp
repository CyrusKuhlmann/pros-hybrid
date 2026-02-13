/**
 * LCD (LLEMU) namespace C++ function stub implementations for desktop
 * simulation. Implements: pros::lcd:: weak-symbol functions
 *
 * IMPORTANT: We intentionally do NOT include pros/llemu.hpp here.
 * That header declares these functions as extern __attribute__((weak)),
 * which on MinGW/GCC causes the definitions to also be emitted as weak symbols,
 * preventing proper linkage. By declaring them ourselves without the weak
 * attribute, we get strong definitions that resolve the weak references.
 */

#include <cstdint>
#include <string>

#include "pros/llemu.h"

namespace pros {
namespace lcd {

using lcd_btn_cb_fn_t = void (*)(void);

bool is_initialized(void) { return true; }

bool initialize(void) { return true; }

bool shutdown(void) { return true; }

bool set_text(std::int16_t line, std::string text) {
  (void)line;
  (void)text;
  return true;
}

bool clear(void) { return true; }

bool clear_line(std::int16_t line) {
  (void)line;
  return true;
}

void register_btn0_cb(lcd_btn_cb_fn_t cb) { (void)cb; }

void register_btn1_cb(lcd_btn_cb_fn_t cb) { (void)cb; }

void register_btn2_cb(lcd_btn_cb_fn_t cb) { (void)cb; }

std::uint8_t read_buttons(void) { return 0; }

}  // namespace lcd
}  // namespace pros
