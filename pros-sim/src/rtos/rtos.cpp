#include "pros/rtos.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "pros/rtos.hpp"

// Simulation time tracking
static auto start_time = std::chrono::steady_clock::now();
static std::atomic<uint32_t> task_count{0};

extern "C" {
namespace pros {

uint32_t millis() {
  auto now = std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time)
          .count());
}

uint32_t micros() {
  auto now = std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now - start_time)
          .count());
}

void delay(uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

task_t task_create(task_fn_t function, void* parameters, uint32_t prio,
                   uint16_t stack_depth, const char* name) {
  (void)prio;         // Ignored on desktop
  (void)stack_depth;  // Ignored on desktop
  (void)name;         // Ignored on desktop

  auto* thread = new std::thread(function, parameters);
  task_count++;
  return static_cast<task_t>(thread);
}

void task_delete(task_t task) {
  if (task) {
    auto* thread = static_cast<std::thread*>(task);
    if (thread->joinable()) {
      thread->join();
    }
    delete thread;
    task_count--;
  }
}

void task_delay(uint32_t milliseconds) { delay(milliseconds); }

uint32_t task_get_count() { return task_count.load(); }

mutex_t mutex_create() { return static_cast<mutex_t>(new std::mutex()); }

void mutex_delete(mutex_t mutex) {
  if (mutex) {
    delete static_cast<std::mutex*>(mutex);
  }
}

bool mutex_take(mutex_t mutex, uint32_t timeout) {
  if (!mutex) return false;

  auto* m = static_cast<std::mutex*>(mutex);
  if (timeout == TIMEOUT_MAX) {
    m->lock();
    return true;
  } else {
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
    while (std::chrono::steady_clock::now() < deadline) {
      if (m->try_lock()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }
}

bool mutex_give(mutex_t mutex) {
  if (!mutex) return false;
  static_cast<std::mutex*>(mutex)->unlock();
  return true;
}

}  // namespace pros
}  // extern "C"

namespace pros {
inline namespace rtos {

Task::Task(task_fn_t function, void* parameters, std::uint32_t prio,
           std::uint16_t stack_depth, const char* name)
    : task(task_create(function, parameters, prio, stack_depth, name)) {}

Task::Task(std::function<void()> function, std::uint32_t prio,
           std::uint16_t stack_depth, const char* name)
    : func_ptr(std::make_shared<std::function<void()>>(std::move(function))) {
  auto wrapper = [](void* param) {
    auto* func = static_cast<std::function<void()>*>(param);
    (*func)();
  };

  task = task_create(wrapper, func_ptr.get(), prio, stack_depth, name);
}

Task::~Task() {
  // Note: Don't automatically delete task on destruction to match PROS behavior
}

void Task::remove() {
  task_delete(task);
  task = nullptr;
}

void Task::delay(std::uint32_t milliseconds) { pros::delay(milliseconds); }

std::uint32_t Task::get_count() { return task_get_count(); }

Mutex::Mutex() : mutex(mutex_create()) {}

Mutex::~Mutex() { mutex_delete(mutex); }

bool Mutex::take(std::uint32_t timeout) { return mutex_take(mutex, timeout); }

bool Mutex::give() { return mutex_give(mutex); }

}  // namespace rtos
}  // namespace pros
