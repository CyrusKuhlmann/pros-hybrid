#include <array>
#include <cstdarg>
#include <cstdio>
#include <iostream>

#include "pros/llemu.h"
#include "pros/llemu.hpp"

static bool initialized = false;
static std::array<std::string, 8> lcd_lines;

extern "C" {
namespace pros {
namespace c {

bool lcd_initialize() {
  initialized = true;
  lcd_lines.fill("");
  std::cout << "[LCD] Initialized\n";
  return true;
}

bool lcd_is_initialized() { return initialized; }

bool lcd_shutdown() {
  initialized = false;
  return true;
}

bool lcd_set_text(int16_t line, const char* text) {
  if (line >= 0 && line < 8) {
    lcd_lines[line] = text;
    std::cout << "[LCD Line " << line << "] " << text << "\n";
    return true;
  }
  return false;
}

bool lcd_clear_line(int16_t line) { return lcd_set_text(line, ""); }

bool lcd_print(int16_t line, const char* fmt, ...) {
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return lcd_set_text(line, buffer);
}

uint8_t lcd_read_buttons() {
  return 0;  // No buttons pressed in simulator
}

bool lcd_register_btn0_cb(lcd_btn_cb_fn_t cb) {
  (void)cb;
  return true;
}

bool lcd_register_btn1_cb(lcd_btn_cb_fn_t cb) {
  (void)cb;
  return true;
}

bool lcd_register_btn2_cb(lcd_btn_cb_fn_t cb) {
  (void)cb;
  return true;
}

}  // namespace c
}  // namespace pros
}  // extern "C"

namespace pros {
namespace lcd {

bool initialize() { return c::lcd_initialize(); }

bool is_initialized() { return c::lcd_is_initialized(); }

bool shutdown() { return c::lcd_shutdown(); }

bool set_text(std::int16_t line, const std::string& text) {
  return c::lcd_set_text(line, text.c_str());
}

bool clear_line(std::int16_t line) { return c::lcd_clear_line(line); }

std::uint8_t read_buttons() { return c::lcd_read_buttons(); }

bool register_btn0_cb(c::lcd_btn_cb_fn_t cb) {
  return c::lcd_register_btn0_cb(cb);
}

bool register_btn1_cb(c::lcd_btn_cb_fn_t cb) {
  return c::lcd_register_btn1_cb(cb);
}

bool register_btn2_cb(c::lcd_btn_cb_fn_t cb) {
  return c::lcd_register_btn2_cb(cb);
}

}  // namespace lcd
}  // namespace pros
