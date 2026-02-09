/**
 * \file api.h
 *
 * Master PROS API header
 */

#ifndef _PROS_API_H_
#define _PROS_API_H_

#ifdef __cplusplus
#include <cerrno>
#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#else
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#endif

#include "pros/device.h"
#include "pros/distance.h"
#include "pros/error.h"
#include "pros/imu.h"
#include "pros/llemu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rtos.h"

#ifdef __cplusplus
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#endif

#endif  // _PROS_API_H_
