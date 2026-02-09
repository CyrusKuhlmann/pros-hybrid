/**
 * \file pros/distance.h
 *
 * Contains C distance sensor functions
 */

#ifndef _PROS_DISTANCE_H_
#define _PROS_DISTANCE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t distance_get(uint8_t port);
int32_t distance_get_confidence(uint8_t port);

#ifdef __cplusplus
}
#endif

#endif  // _PROS_DISTANCE_H_
