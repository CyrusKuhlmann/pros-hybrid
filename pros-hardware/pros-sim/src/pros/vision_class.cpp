/**
 * PROS Vision C++ class stub implementations for simulation
 */
#include "pros/vision.hpp"

namespace pros {
  inline namespace v5 {

    Vision::Vision(const std::uint8_t port, vision_zero_e_t zero_point)
      : Device(port, DeviceType::vision) {
      (void)zero_point;
    }

    std::int32_t Vision::clear_led() const { return 1; }

    vision_signature_s_t Vision::signature_from_utility(
      const std::int32_t id, const std::int32_t u_min, const std::int32_t u_max,
      const std::int32_t u_mean, const std::int32_t v_min,
      const std::int32_t v_max, const std::int32_t v_mean, const float range,
      const std::int32_t type) {
      vision_signature_s_t sig = {};
      sig.id = id;
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

    vision_color_code_t Vision::create_color_code(
      const std::uint32_t sig_id1, const std::uint32_t sig_id2,
      const std::uint32_t sig_id3, const std::uint32_t sig_id4,
      const std::uint32_t sig_id5) const {
      (void)sig_id1;
      (void)sig_id2;
      (void)sig_id3;
      (void)sig_id4;
      (void)sig_id5;
      return 0;
    }

    std::vector<Vision> Vision::get_all_devices() { return {}; }
    Vision Vision::get_vision() { return Vision(1); }

    vision_object_s_t Vision::get_by_size(const std::uint32_t size_id) const {
      (void)size_id;
      vision_object_s_t o = {};
      return o;
    }
    vision_object_s_t Vision::get_by_sig(const std::uint32_t size_id,
      const std::uint32_t sig_id) const {
      (void)size_id;
      (void)sig_id;
      vision_object_s_t o = {};
      return o;
    }
    vision_object_s_t Vision::get_by_code(
      const std::uint32_t size_id, const vision_color_code_t color_code) const {
      (void)size_id;
      (void)color_code;
      vision_object_s_t o = {};
      return o;
    }
    std::int32_t Vision::get_exposure() const { return 0; }
    std::int32_t Vision::get_object_count() const { return 0; }
    vision_signature_s_t Vision::get_signature(const std::uint8_t sig_id) const {
      (void)sig_id;
      vision_signature_s_t s = {};
      return s;
    }
    std::int32_t Vision::get_white_balance() const { return 0; }
    std::int32_t Vision::read_by_size(const std::uint32_t size_id,
      const std::uint32_t object_count,
      vision_object_s_t* const object_arr) const {
      (void)size_id;
      (void)object_count;
      (void)object_arr;
      return 0;
    }
    std::int32_t Vision::read_by_sig(const std::uint32_t size_id,
      const std::uint32_t sig_id,
      const std::uint32_t object_count,
      vision_object_s_t* const object_arr) const {
      (void)size_id;
      (void)sig_id;
      (void)object_count;
      (void)object_arr;
      return 0;
    }
    std::int32_t Vision::read_by_code(const std::uint32_t size_id,
      const vision_color_code_t color_code,
      const std::uint32_t object_count,
      vision_object_s_t* const object_arr) const {
      (void)size_id;
      (void)color_code;
      (void)object_count;
      (void)object_arr;
      return 0;
    }
    std::int32_t Vision::print_signature(const vision_signature_s_t sig) {
      (void)sig;
      return 1;
    }
    std::int32_t Vision::set_auto_white_balance(const std::uint8_t enable) const {
      (void)enable;
      return 1;
    }
    std::int32_t Vision::set_exposure(const std::uint8_t exposure) const {
      (void)exposure;
      return 1;
    }
    std::int32_t Vision::set_led(const std::int32_t rgb) const {
      (void)rgb;
      return 1;
    }
    std::int32_t Vision::set_signature(const std::uint8_t sig_id,
      vision_signature_s_t* const sig_ptr) const {
      (void)sig_id;
      (void)sig_ptr;
      return 1;
    }
    std::int32_t Vision::set_white_balance(const std::int32_t rgb) const {
      (void)rgb;
      return 1;
    }
    std::int32_t Vision::set_zero_point(const vision_zero_e_t zero_point) const {
      (void)zero_point;
      return 1;
    }
    std::int32_t Vision::set_wifi_mode(const std::uint8_t enable) const {
      (void)enable;
      return 1;
    }

    namespace literals {
      const Vision operator""_vis(const unsigned long long int v) {
        return Vision((std::uint8_t)v);
      }
    }  // namespace literals

  }  // namespace v5
}  // namespace pros
