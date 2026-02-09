/**
 * \file pros/llemu.hpp
 *
 * Contains LLEMU (LCD emulator) C++ namespace
 */

#ifndef _PROS_LLEMU_HPP_
#define _PROS_LLEMU_HPP_

#include <cstdint>
#include <string>

#include "pros/llemu.h"

namespace pros {
namespace lcd {

bool initialize();
bool is_initialized();
bool shutdown();
bool set_text(std::int16_t line, const std::string& text);
bool clear_line(std::int16_t line);
std::uint8_t read_buttons();
bool register_btn0_cb(c::lcd_btn_cb_fn_t cb);
bool register_btn1_cb(c::lcd_btn_cb_fn_t cb);
bool register_btn2_cb(c::lcd_btn_cb_fn_t cb);

// Variadic template for print
template <typename... Args>
bool print(std::int16_t line, const char* fmt, Args... args) {
  return c::lcd_print(line, fmt, args...);
}

}  // namespace lcd
}  // namespace pros

#endif  // _PROS_LLEMU_HPP_
