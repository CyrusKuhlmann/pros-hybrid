/**
 * \file pros/rtos.h
 *
 * Contains C RTOS functions
 */

#ifndef _PROS_RTOS_H_
#define _PROS_RTOS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
namespace pros {
#endif

#define TASK_PRIORITY_MAX 16
#define TASK_PRIORITY_MIN 1
#define TASK_PRIORITY_DEFAULT 8
#define TASK_STACK_DEPTH_DEFAULT 0x2000
#define TASK_STACK_DEPTH_MIN 0x200
#define TASK_NAME_MAX_LEN 32
#define TIMEOUT_MAX ((uint32_t)0xffffffffUL)

#ifdef __cplusplus
#define CURRENT_TASK ((pros::task_t)NULL)
#else
#define CURRENT_TASK ((task_t)NULL)
#endif

typedef void* task_t;
typedef void* mutex_t;
typedef void (*task_fn_t)(void*);

typedef enum {
  E_TASK_STATE_RUNNING = 0,
  E_TASK_STATE_READY,
  E_TASK_STATE_BLOCKED,
  E_TASK_STATE_SUSPENDED,
  E_TASK_STATE_DELETED,
  E_TASK_STATE_INVALID
} task_state_e_t;

typedef enum {
  E_NOTIFY_ACTION_NONE,
  E_NOTIFY_ACTION_BITS,
  E_NOTIFY_ACTION_INCR,
  E_NOTIFY_ACTION_OWRITE,
  E_NOTIFY_ACTION_NO_OWRITE
} notify_action_e_t;

#ifdef PROS_USE_SIMPLE_NAMES
#ifdef __cplusplus
#define TASK_STATE_RUNNING pros::E_TASK_STATE_RUNNING
#define TASK_STATE_READY pros::E_TASK_STATE_READY
#define TASK_STATE_BLOCKED pros::E_TASK_STATE_BLOCKED
#define TASK_STATE_SUSPENDED pros::E_TASK_STATE_SUSPENDED
#define TASK_STATE_DELETED pros::E_TASK_STATE_DELETED
#define TASK_STATE_INVALID pros::E_TASK_STATE_INVALID
#define NOTIFY_ACTION_NONE pros::E_NOTIFY_ACTION_NONE
#define NOTIFY_ACTION_BITS pros::E_NOTIFY_ACTION_BITS
#define NOTIFY_ACTION_INCR pros::E_NOTIFY_ACTION_INCR
#define NOTIFY_ACTION_OWRITE pros::E_NOTIFY_ACTION_OWRITE
#define NOTIFY_ACTION_NO_OWRITE pros::E_NOTIFY_ACTION_NO_OWRITE
#else
#define TASK_STATE_RUNNING E_TASK_STATE_RUNNING
#define TASK_STATE_READY E_TASK_STATE_READY
#define TASK_STATE_BLOCKED E_TASK_STATE_BLOCKED
#define TASK_STATE_SUSPENDED E_TASK_STATE_SUSPENDED
#define TASK_STATE_DELETED E_TASK_STATE_DELETED
#define TASK_STATE_INVALID E_TASK_STATE_INVALID
#define NOTIFY_ACTION_NONE E_NOTIFY_ACTION_NONE
#define NOTIFY_ACTION_BITS E_NOTIFY_ACTION_BITS
#define NOTIFY_ACTION_INCR E_NOTIFY_ACTION_INCR
#define NOTIFY_ACTION_OWRITE E_NOTIFY_ACTION_OWRITE
#define NOTIFY_ACTION_NO_OWRITE E_NOTIFY_ACTION_NO_OWRITE
#endif
#endif

uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t milliseconds);

task_t task_create(task_fn_t function, void* parameters, uint32_t prio,
                   uint16_t stack_depth, const char* name);
void task_delete(task_t task);
void task_delay(uint32_t milliseconds);
uint32_t task_get_count(void);

mutex_t mutex_create(void);
void mutex_delete(mutex_t mutex);
bool mutex_take(mutex_t mutex, uint32_t timeout);
bool mutex_give(mutex_t mutex);

#ifdef __cplusplus
}
}  // namespace pros
#endif

#endif  // _PROS_RTOS_H_
