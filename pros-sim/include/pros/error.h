/**
 * \file pros/error.h
 *
 * Contains error codes and errno values
 */

#ifndef _PROS_ERROR_H_
#define _PROS_ERROR_H_

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROS_ERR_BYTE (INT8_MAX)
#define PROS_ERR_2_BYTE (INT16_MAX)
#define PROS_ERR (INT32_MAX)
#define PROS_ERR_F (INFINITY)
#define PROS_SUCCESS (1)

// Error codes match standard errno values
#define ENODEV 19  // No such device
#define ENXIO 6    // Port out of range
#define EACCES 13  // Access denied
#define EAGAIN 11  // Resource temporarily unavailable

#ifdef __cplusplus
}
#endif

#endif  // _PROS_ERROR_H_
