/**
 * PROS Vision C API stub implementations for simulation
 */
#include "pros/vision.h"

extern "C" {
  namespace pros {
    namespace c {

      int32_t vision_clear_led(uint8_t port) {
        (void)port;
        return 1;
      }

      vision_signature_s_t vision_signature_from_utility(
        const int32_t id, const int32_t u_min, const int32_t u_max,
        const int32_t u_mean, const int32_t v_min, const int32_t v_max,
        const int32_t v_mean, const float range, const int32_t type) {
        vision_signature_s_t sig = {};
        sig.id = (uint8_t)id;
        sig.u_min = u_min;
        sig.u_max = u_max;
        sig.u_mean = u_mean;
        sig.v_min = v_min;
        sig.v_max = v_max;
        sig.v_mean = v_mean;
        sig.range = range;
        sig.type = (uint32_t)type;
        return sig;
      }

      vision_color_code_t vision_create_color_code(
        uint8_t port, const uint32_t sig_id1, const uint32_t sig_id2,
        const uint32_t sig_id3, const uint32_t sig_id4, const uint32_t sig_id5) {
        (void)port;
        (void)sig_id1;
        (void)sig_id2;
        (void)sig_id3;
        (void)sig_id4;
        (void)sig_id5;
        return 0;
      }

      vision_object_s_t vision_get_by_size(uint8_t port, const uint32_t size_id) {
        (void)port;
        (void)size_id;
        vision_object_s_t o = {};
        return o;
      }

      vision_object_s_t vision_get_by_sig(uint8_t port, const uint32_t size_id,
        const uint32_t sig_id) {
        (void)port;
        (void)size_id;
        (void)sig_id;
        vision_object_s_t o = {};
        return o;
      }

      vision_object_s_t vision_get_by_code(uint8_t port, const uint32_t size_id,
        const vision_color_code_t color_code) {
        (void)port;
        (void)size_id;
        (void)color_code;
        vision_object_s_t o = {};
        return o;
      }

      int32_t vision_get_exposure(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t vision_get_object_count(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t vision_get_white_balance(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t vision_print_signature(const vision_signature_s_t sig) {
        (void)sig;
        return 1;
      }

      int32_t vision_read_by_size(uint8_t port, const uint32_t size_id,
        const uint32_t object_count,
        vision_object_s_t* const object_arr) {
        (void)port;
        (void)size_id;
        (void)object_count;
        (void)object_arr;
        return 0;
      }

      int32_t vision_read_by_sig(uint8_t port, const uint32_t size_id,
        const uint32_t sig_id, const uint32_t object_count,
        vision_object_s_t* const object_arr) {
        (void)port;
        (void)size_id;
        (void)sig_id;
        (void)object_count;
        (void)object_arr;
        return 0;
      }

      int32_t vision_read_by_code(uint8_t port, const uint32_t size_id,
        const vision_color_code_t color_code,
        const uint32_t object_count,
        vision_object_s_t* const object_arr) {
        (void)port;
        (void)size_id;
        (void)color_code;
        (void)object_count;
        (void)object_arr;
        return 0;
      }

      vision_signature_s_t vision_get_signature(uint8_t port,
        const uint8_t signature_id) {
        (void)port;
        (void)signature_id;
        vision_signature_s_t s = {};
        return s;
      }

      int32_t vision_set_signature(uint8_t port, const uint8_t signature_id,
        vision_signature_s_t* const signature_ptr) {
        (void)port;
        (void)signature_id;
        (void)signature_ptr;
        return 1;
      }

      int32_t vision_set_auto_white_balance(uint8_t port, const uint8_t enable) {
        (void)port;
        (void)enable;
        return 1;
      }
      int32_t vision_set_exposure(uint8_t port, const uint8_t exposure) {
        (void)port;
        (void)exposure;
        return 1;
      }
      int32_t vision_set_led(uint8_t port, const int32_t rgb) {
        (void)port;
        (void)rgb;
        return 1;
      }
      int32_t vision_set_white_balance(uint8_t port, const int32_t rgb) {
        (void)port;
        (void)rgb;
        return 1;
      }
      int32_t vision_set_zero_point(uint8_t port, vision_zero_e_t zero_point) {
        (void)port;
        (void)zero_point;
        return 1;
      }
      int32_t vision_set_wifi_mode(uint8_t port, const uint8_t enable) {
        (void)port;
        (void)enable;
        return 1;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
