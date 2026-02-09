/**
 * \file pros/device.h
 *
 * Contains C device type enum
 */

#ifndef _PROS_DEVICE_H_
#define _PROS_DEVICE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum v5_device_e {
  E_DEVICE_NONE = 0,
  E_DEVICE_MOTOR = 2,
  E_DEVICE_ROTATION = 4,
  E_DEVICE_IMU = 6,
  E_DEVICE_DISTANCE = 7,
  E_DEVICE_RADIO = 8,
  E_DEVICE_VISION = 11,
  E_DEVICE_ADI = 12,
  E_DEVICE_OPTICAL = 16,
  E_DEVICE_GPS = 20,
  E_DEVICE_AIVISION = 29,
  E_DEVICE_SERIAL = 129,
  E_DEVICE_GENERIC = E_DEVICE_SERIAL,
  E_DEVICE_UNDEFINED = 255
} v5_device_e_t;

#ifdef __cplusplus
}
#endif

#endif  // _PROS_DEVICE_H_
