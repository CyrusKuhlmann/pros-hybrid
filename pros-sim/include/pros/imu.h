/**
 * \file pros/imu.h
 *
 * Contains C IMU functions
 */

#ifndef _PROS_IMU_H_
#define _PROS_IMU_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
namespace pros {
#endif

#define IMU_MINIMUM_DATA_RATE 5

typedef enum imu_status_e {
  E_IMU_STATUS_READY = 0,
  E_IMU_STATUS_CALIBRATING = 1,
  E_IMU_STATUS_ERROR = 0xFF,
} imu_status_e_t;

typedef enum imu_orientation_e {
  E_IMU_Z_UP = 0,
  E_IMU_Z_DOWN = 1,
  E_IMU_X_UP = 2,
  E_IMU_X_DOWN = 3,
  E_IMU_Y_UP = 4,
  E_IMU_Y_DOWN = 5,
  E_IMU_ORIENTATION_ERROR = 0xFF
} imu_orientation_e_t;

typedef struct quaternion_s {
  double x;
  double y;
  double z;
  double w;
} quaternion_s_t;

struct imu_raw_s {
  double x;
  double y;
  double z;
};

typedef struct imu_raw_s imu_gyro_s_t;
typedef struct imu_raw_s imu_accel_s_t;

typedef struct euler_s {
  double pitch;
  double roll;
  double yaw;
} euler_s_t;

#ifdef PROS_USE_SIMPLE_NAMES
#ifdef __cplusplus
#define IMU_STATUS_CALIBRATING pros::E_IMU_STATUS_CALIBRATING
#define IMU_STATUS_ERROR pros::E_IMU_STATUS_ERROR
#else
#define IMU_STATUS_CALIBRATING E_IMU_STATUS_CALIBRATING
#define IMU_STATUS_ERROR E_IMU_STATUS_ERROR
#endif
#endif

int32_t imu_reset(uint8_t port);
double imu_get_heading(uint8_t port);
double imu_get_rotation(uint8_t port);
int32_t imu_tare_heading(uint8_t port);
int32_t imu_tare_rotation(uint8_t port);
int32_t imu_set_heading(uint8_t port, double target);
int32_t imu_set_rotation(uint8_t port, double target);

#ifdef __cplusplus
}
}  // namespace pros
#endif

#endif  // _PROS_IMU_H_
