/**
 * PROS RTOS C++ class stub implementations for simulation (Task, Mutex, Clock)
 */
#include <chrono>
#include <mutex>
#include <thread>

#include "pros/rtos.hpp"

namespace pros {
inline namespace rtos {

// --- Task ---
Task::Task(task_fn_t fn, void* param, std::uint32_t prio,
           std::uint16_t stack_depth, const char* name)
    : task(static_cast<task_t>(nullptr)) {
  (void)prio;
  (void)stack_depth;
  (void)name;
  if (fn) {
    std::thread(fn, param).detach();
  }
}

Task::Task(task_fn_t fn, void* param, const char* name)
    : task(static_cast<task_t>(nullptr)) {
  (void)name;
  if (fn) {
    std::thread(fn, param).detach();
  }
}

Task::Task(task_t task) : task(task) {}

Task Task::current() { return Task(static_cast<task_t>(nullptr)); }

Task& Task::operator=(task_t in_task) {
  task = in_task;
  return *this;
}

void Task::remove() {}
std::uint32_t Task::get_priority() { return TASK_PRIORITY_DEFAULT; }
void Task::set_priority(std::uint32_t priority) { (void)priority; }
std::uint32_t Task::get_state() { return E_TASK_STATE_RUNNING; }
void Task::suspend() {}
void Task::resume() {}
const char* Task::get_name() { return "sim_task"; }
std::uint32_t Task::notify() { return 1; }
void Task::join() {}
std::uint32_t Task::notify_ext(std::uint32_t value, notify_action_e_t action,
                               std::uint32_t* prev_value) {
  (void)value;
  (void)action;
  if (prev_value) *prev_value = 0;
  return 0;
}
std::uint32_t Task::notify_take(bool clear_on_exit, std::uint32_t timeout) {
  (void)clear_on_exit;
  (void)timeout;
  return 0;
}
bool Task::notify_clear() { return true; }

void Task::delay(const std::uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void Task::delay_until(std::uint32_t* const prev_time,
                       const std::uint32_t delta) {
  pros::c::task_delay_until(prev_time, delta);
}

std::uint32_t Task::get_count() { return 1; }

// --- Clock ---
Clock::time_point Clock::now() {
  return time_point(duration(pros::c::millis()));
}

// --- Mutex ---
static std::mutex s_mutex_backing;

Mutex::~Mutex() {}
mutex_t Mutex::lazy_init() { return nullptr; }
bool Mutex::take() { return true; }
bool Mutex::take(std::uint32_t timeout) {
  (void)timeout;
  return true;
}
bool Mutex::give() { return true; }
void Mutex::lock() {}
void Mutex::unlock() {}
bool Mutex::try_lock() { return true; }

// --- RecursiveMutex ---
RecursiveMutex::~RecursiveMutex() {}
mutex_t RecursiveMutex::lazy_init() { return nullptr; }
bool RecursiveMutex::take() { return true; }
bool RecursiveMutex::take(std::uint32_t timeout) {
  (void)timeout;
  return true;
}
bool RecursiveMutex::give() { return true; }
void RecursiveMutex::lock() {}
void RecursiveMutex::unlock() {}
bool RecursiveMutex::try_lock() { return true; }

}  // namespace rtos
}  // namespace pros
