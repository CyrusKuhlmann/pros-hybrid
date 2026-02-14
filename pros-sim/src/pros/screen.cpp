/**
 * PROS Screen C API stub implementations for simulation
 */
#include "pros/screen.h"

#include <cstdarg>

extern "C" {
  namespace pros {
    namespace c {

      uint32_t screen_set_pen(uint32_t color) {
        (void)color;
        return 1;
      }
      uint32_t screen_set_eraser(uint32_t color) {
        (void)color;
        return 1;
      }
      uint32_t screen_get_pen(void) { return 0xFFFFFF; }
      uint32_t screen_get_eraser(void) { return 0x000000; }
      uint32_t screen_erase(void) { return 1; }
      uint32_t screen_scroll(int16_t start_line, int16_t lines) {
        (void)start_line;
        (void)lines;
        return 1;
      }
      uint32_t screen_scroll_area(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
        int16_t lines) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        (void)lines;
        return 1;
      }
      uint32_t screen_copy_area(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
        uint32_t* buf, int32_t stride) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        (void)buf;
        (void)stride;
        return 1;
      }
      uint32_t screen_draw_pixel(int16_t x, int16_t y) {
        (void)x;
        (void)y;
        return 1;
      }
      uint32_t screen_erase_pixel(int16_t x, int16_t y) {
        (void)x;
        (void)y;
        return 1;
      }
      uint32_t screen_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        return 1;
      }
      uint32_t screen_erase_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        return 1;
      }
      uint32_t screen_draw_rect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        return 1;
      }
      uint32_t screen_erase_rect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        return 1;
      }
      uint32_t screen_fill_rect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        (void)x0;
        (void)y0;
        (void)x1;
        (void)y1;
        return 1;
      }
      uint32_t screen_draw_circle(int16_t x, int16_t y, int16_t radius) {
        (void)x;
        (void)y;
        (void)radius;
        return 1;
      }
      uint32_t screen_erase_circle(int16_t x, int16_t y, int16_t radius) {
        (void)x;
        (void)y;
        (void)radius;
        return 1;
      }
      uint32_t screen_fill_circle(int16_t x, int16_t y, int16_t radius) {
        (void)x;
        (void)y;
        (void)radius;
        return 1;
      }
      uint32_t screen_print(text_format_e_t txt_fmt, const int16_t line,
        const char* text, ...) {
        (void)txt_fmt;
        (void)line;
        (void)text;
        return 1;
      }
      uint32_t screen_print_at(text_format_e_t txt_fmt, const int16_t x,
        const int16_t y, const char* text, ...) {
        (void)txt_fmt;
        (void)x;
        (void)y;
        (void)text;
        return 1;
      }
      uint32_t screen_vprintf(text_format_e_t txt_fmt, const int16_t line,
        const char* text, va_list args) {
        (void)txt_fmt;
        (void)line;
        (void)text;
        (void)args;
        return 1;
      }
      uint32_t screen_vprintf_at(text_format_e_t txt_fmt, const int16_t x,
        const int16_t y, const char* text, va_list args) {
        (void)txt_fmt;
        (void)x;
        (void)y;
        (void)text;
        (void)args;
        return 1;
      }

      screen_touch_status_s_t screen_touch_status(void) {
        screen_touch_status_s_t s = {};
        s.touch_status = E_TOUCH_RELEASED;
        return s;
      }

      uint32_t screen_touch_callback(touch_event_cb_fn_t cb,
        last_touch_e_t event_type) {
        (void)cb;
        (void)event_type;
        return 1;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
