/**
 * \file pros/rotation.h
 *
 * Contains C rotation sensor functions
 */

#ifndef _PROS_ROTATION_H_
#define _PROS_ROTATION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROTATION_MINIMUM_DATA_RATE 5

int32_t rotation_reset(uint8_t port);
int32_t rotation_get_position(uint8_t port);
int32_t rotation_get_velocity(uint8_t port);
int32_t rotation_set_position(uint8_t port, uint32_t position);
int32_t rotation_reset_position(uint8_t port);

#ifdef __cplusplus
}
#endif

#endif  // _PROS_ROTATION_H_
