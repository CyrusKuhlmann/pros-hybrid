/**
 * \file pros/rtos.hpp
 *
 * Contains C++ RTOS wrapper classes
 */

#ifndef _PROS_RTOS_HPP_
#define _PROS_RTOS_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "pros/rtos.h"

namespace pros {
inline namespace rtos {

class Task {
 public:
  Task(task_fn_t function, void* parameters = nullptr,
       std::uint32_t prio = TASK_PRIORITY_DEFAULT,
       std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT,
       const char* name = "");

  Task(std::function<void()> function,
       std::uint32_t prio = TASK_PRIORITY_DEFAULT,
       std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT,
       const char* name = "");

  ~Task();

  void remove();
  static void delay(std::uint32_t milliseconds);
  static std::uint32_t get_count();

 private:
  task_t task;
  std::shared_ptr<std::function<void()>> func_ptr;
};

class Mutex {
 public:
  Mutex();
  ~Mutex();

  bool take(std::uint32_t timeout = TIMEOUT_MAX);
  bool give();

 private:
  mutex_t mutex;
};

}  // namespace rtos

// Free functions in pros namespace (already declared via rtos.h)
// using pros::delay;
// using pros::micros;
// using pros::millis;

}  // namespace pros

#endif  // _PROS_RTOS_HPP_
