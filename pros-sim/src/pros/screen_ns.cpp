/**
 * Screen namespace C++ function stub implementations for desktop simulation.
 * Implements: pros::screen:: free functions (non-template)
 */

#include "pros/screen.hpp"

namespace pros {
  namespace screen {

    std::uint32_t set_pen(pros::Color color) {
      (void)color;
      return 1;
    }

    std::uint32_t set_pen(std::uint32_t color) {
      (void)color;
      return 1;
    }

    std::uint32_t set_eraser(pros::Color color) {
      (void)color;
      return 1;
    }

    std::uint32_t set_eraser(std::uint32_t color) {
      (void)color;
      return 1;
    }

    std::uint32_t get_pen() { return 0xFFFFFF; }

    std::uint32_t get_eraser() { return 0x000000; }

    std::uint32_t erase() { return 1; }

    std::uint32_t scroll(const std::int16_t start_line, const std::int16_t lines) {
      (void)start_line;
      (void)lines;
      return 1;
    }

    std::uint32_t scroll_area(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1,
      std::int16_t lines) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      (void)lines;
      return 1;
    }

    std::uint32_t copy_area(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1,
      uint32_t* buf, const std::int32_t stride) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      (void)buf;
      (void)stride;
      return 1;
    }

    std::uint32_t draw_pixel(const std::int16_t x, const std::int16_t y) {
      (void)x;
      (void)y;
      return 1;
    }

    std::uint32_t erase_pixel(const std::int16_t x, const std::int16_t y) {
      (void)x;
      (void)y;
      return 1;
    }

    std::uint32_t draw_line(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      return 1;
    }

    std::uint32_t erase_line(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      return 1;
    }

    std::uint32_t draw_rect(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      return 1;
    }

    std::uint32_t erase_rect(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      return 1;
    }

    std::uint32_t fill_rect(const std::int16_t x0, const std::int16_t y0,
      const std::int16_t x1, const std::int16_t y1) {
      (void)x0;
      (void)y0;
      (void)x1;
      (void)y1;
      return 1;
    }

    std::uint32_t draw_circle(const std::int16_t x, const std::int16_t y,
      const std::int16_t radius) {
      (void)x;
      (void)y;
      (void)radius;
      return 1;
    }

    std::uint32_t erase_circle(const std::int16_t x, const std::int16_t y,
      const std::int16_t radius) {
      (void)x;
      (void)y;
      (void)radius;
      return 1;
    }

    std::uint32_t fill_circle(const std::int16_t x, const std::int16_t y,
      const std::int16_t radius) {
      (void)x;
      (void)y;
      (void)radius;
      return 1;
    }

    screen_touch_status_s_t touch_status() {
      screen_touch_status_s_t status;
      status.touch_status = E_TOUCH_RELEASED;
      status.x = 0;
      status.y = 0;
      status.press_count = 0;
      status.release_count = 0;
      return status;
    }

    std::uint32_t touch_callback(touch_event_cb_fn_t cb,
      last_touch_e_t event_type) {
      (void)cb;
      (void)event_type;
      return 1;
    }

  }  // namespace screen
}  // namespace pros
