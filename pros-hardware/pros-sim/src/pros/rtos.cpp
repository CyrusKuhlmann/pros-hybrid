/**
 * PROS RTOS C API stub implementations for simulation
 */
#include "pros/rtos.h"

#include <chrono>
#include <cstring>
#include <thread>

static auto start_time = std::chrono::steady_clock::now();

extern "C" {
namespace pros {
namespace c {

uint32_t millis(void) {
  auto now = std::chrono::steady_clock::now();
  return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
             now - start_time)
      .count();
}

uint64_t micros(void) {
  auto now = std::chrono::steady_clock::now();
  return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
             now - start_time)
      .count();
}

task_t task_create(task_fn_t function, void* const parameters, uint32_t prio,
                   const uint16_t stack_depth, const char* const name) {
  (void)function;
  (void)parameters;
  (void)prio;
  (void)stack_depth;
  (void)name;
  return nullptr;
}

void task_delete(task_t task) { (void)task; }

void task_delay(const uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void delay(const uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void task_delay_until(uint32_t* const prev_time, const uint32_t delta) {
  if (prev_time) {
    uint32_t target = *prev_time + delta;
    uint32_t now = millis();
    if (target > now) {
      std::this_thread::sleep_for(std::chrono::milliseconds(target - now));
    }
    *prev_time = millis();
  }
}

uint32_t task_get_priority(task_t task) {
  (void)task;
  return TASK_PRIORITY_DEFAULT;
}
void task_set_priority(task_t task, uint32_t prio) {
  (void)task;
  (void)prio;
}
task_state_e_t task_get_state(task_t task) {
  (void)task;
  return E_TASK_STATE_RUNNING;
}
void task_suspend(task_t task) { (void)task; }
void task_resume(task_t task) { (void)task; }
uint32_t task_get_count(void) { return 1; }
char* task_get_name(task_t task) {
  (void)task;
  static char name[] = "sim_task";
  return name;
}
task_t task_get_by_name(const char* name) {
  (void)name;
  return nullptr;
}
task_t task_get_current() { return nullptr; }
uint32_t task_notify(task_t task) {
  (void)task;
  return 1;
}
void task_join(task_t task) { (void)task; }

uint32_t task_notify_ext(task_t task, uint32_t value, notify_action_e_t action,
                         uint32_t* prev_value) {
  (void)task;
  (void)value;
  (void)action;
  if (prev_value) *prev_value = 0;
  return 0;
}

uint32_t task_notify_take(bool clear_on_exit, uint32_t timeout) {
  (void)clear_on_exit;
  (void)timeout;
  return 0;
}

bool task_notify_clear(task_t task) {
  (void)task;
  return true;
}

mutex_t mutex_create(void) {
  return (mutex_t)1;  // Non-null sentinel
}

bool mutex_take(mutex_t mutex, uint32_t timeout) {
  (void)mutex;
  (void)timeout;
  return true;
}
bool mutex_give(mutex_t mutex) {
  (void)mutex;
  return true;
}
mutex_t mutex_recursive_create(void) { return (mutex_t)1; }
bool mutex_recursive_take(mutex_t mutex, uint32_t timeout) {
  (void)mutex;
  (void)timeout;
  return true;
}
bool mutex_recursive_give(mutex_t mutex) {
  (void)mutex;
  return true;
}
void mutex_delete(mutex_t mutex) { (void)mutex; }

}  // namespace c
}  // namespace pros
}  // extern "C"
