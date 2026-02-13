/**
 * PROS AI Vision C API stub implementations for simulation
 */
#include "pros/ai_vision.h"

#include <cstring>

extern "C" {
  namespace pros {
    namespace c {

      int32_t aivision_reset(uint8_t port) {
        (void)port;
        return 1;
      }
      int32_t aivision_get_enabled_detection_types(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t aivision_set_enabled_detection_types(uint8_t port, uint8_t bits,
        uint8_t bitmask) {
        (void)port;
        (void)bits;
        (void)bitmask;
        return 1;
      }
      int32_t aivision_enable_detection_types(uint8_t port, uint8_t types_mask) {
        (void)port;
        (void)types_mask;
        return 1;
      }
      int32_t aivision_disable_detection_types(uint8_t port, uint8_t types_mask) {
        (void)port;
        (void)types_mask;
        return 1;
      }
      int32_t aivision_set_tag_family_override(uint8_t port,
        aivision_tag_family_e_t family) {
        (void)port;
        (void)family;
        return 1;
      }
      int32_t aivision_set_tag_family(uint8_t port, aivision_tag_family_e_t family) {
        (void)port;
        (void)family;
        return 1;
      }
      int32_t aivision_set_color(uint8_t port, const aivision_color_s_t* color) {
        (void)port;
        (void)color;
        return 1;
      }

      aivision_color_s_t aivision_get_color(uint8_t port, uint32_t id) {
        (void)port;
        (void)id;
        aivision_color_s_t c = {};
        return c;
      }

      int32_t aivision_get_class_name(uint8_t port, int32_t id, uint8_t* class_name) {
        (void)port;
        (void)id;
        if (class_name) class_name[0] = 0;
        return 1;
      }

      int32_t aivision_set_usb_bounding_box_overlay(uint8_t port, bool enabled) {
        (void)port;
        (void)enabled;
        return 1;
      }
      int32_t aivision_start_awb(uint8_t port) {
        (void)port;
        return 1;
      }

      aivision_code_s_t aivision_get_code(uint8_t port, uint32_t id) {
        (void)port;
        (void)id;
        aivision_code_s_t c = {};
        return c;
      }

      int32_t aivision_set_code(uint8_t port, const aivision_code_s_t* wcode) {
        (void)port;
        (void)wcode;
        return 1;
      }
      int32_t aivision_get_object_count(uint8_t port) {
        (void)port;
        return 0;
      }

      aivision_object_s_t aivision_get_object(uint8_t port, uint32_t object_index) {
        (void)port;
        (void)object_index;
        aivision_object_s_t o = {};
        return o;
      }

      double aivision_get_temperature(uint8_t port) {
        (void)port;
        return 0.0;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
