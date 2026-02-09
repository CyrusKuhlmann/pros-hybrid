/**
 * \file pros/llemu.h
 *
 * Contains LLEMU (LCD emulator) C functions
 */

#ifndef _PROS_LLEMU_H_
#define _PROS_LLEMU_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
namespace pros {
namespace c {
#endif

#define LCD_BTN_LEFT 4
#define LCD_BTN_CENTER 2
#define LCD_BTN_RIGHT 1

typedef void (*lcd_btn_cb_fn_t)(void);

bool lcd_initialize(void);
bool lcd_is_initialized(void);
bool lcd_shutdown(void);
bool lcd_set_text(int16_t line, const char* text);
bool lcd_clear_line(int16_t line);
bool lcd_print(int16_t line, const char* fmt, ...);
uint8_t lcd_read_buttons(void);
bool lcd_register_btn0_cb(lcd_btn_cb_fn_t cb);
bool lcd_register_btn1_cb(lcd_btn_cb_fn_t cb);
bool lcd_register_btn2_cb(lcd_btn_cb_fn_t cb);

#ifdef __cplusplus
}
}  // namespace c
}  // namespace pros
#endif

#endif  // _PROS_LLEMU_H_
