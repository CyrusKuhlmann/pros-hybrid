#include "actor.h"

#include <algorithm>
#include <cmath>

// Helper to ensure clamp works across toolchains
template <typename T>
T clampValue(T val, T lo, T hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

// ========== Constructor ==========

Actor::Actor(Odom& odom_ref, pros::MotorGroup& left_motors_ref,
             pros::MotorGroup& right_motors_ref,
             MotionControllerSettings lateral_settings,
             MotionControllerSettings angular_settings)
    : odom(odom_ref),
      left_motors(left_motors_ref),
      right_motors(right_motors_ref),
      lateralPID(lateral_settings.kP, lateral_settings.kI, lateral_settings.kD,
                 lateral_settings.windupRange, false),
      angularPID(angular_settings.kP, angular_settings.kI, angular_settings.kD,
                 angular_settings.windupRange, true),
      lateralSettings(lateral_settings),
      angularSettings(angular_settings),
      lateralSmallExit(lateral_settings.smallError,
                       lateral_settings.smallErrorTimeout),
      lateralLargeExit(lateral_settings.largeError,
                       lateral_settings.largeErrorTimeout),
      angularSmallExit(angular_settings.smallError,
                       angular_settings.smallErrorTimeout),
      angularLargeExit(angular_settings.largeError,
                       angular_settings.largeErrorTimeout) {}

// ========== Helper Functions ==========

float Actor::slew(float target, float current, float maxChange) {
  if (maxChange == 0) return target;
  float change = target - current;
  if (change > maxChange) return current + maxChange;
  if (change < -maxChange) return current - maxChange;
  return target;
}

void Actor::tank(float left, float right) {
  left_motors.move(left / 100.0f * 127.0f);
  right_motors.move(right / 100.0f * 127.0f);
}

void Actor::arcade(float throttle, float turn) {
  float left = throttle + turn;
  float right = throttle - turn;
  tank(left, right);
}

float Actor::angleError(float target, float current) {
  // Calculate shortest angle error (-180 to 180)
  float error = std::fmod(target - current + 180.0f, 360.0f);
  if (error < 0) error += 360.0f;
  return error - 180.0f;
}

Pose Actor::getPose() {
  Eigen::Vector2d xy = odom.get_xy_inches();
  return Pose(xy(0), xy(1), odom.get_theta_degrees());
}

void Actor::resetMotionState() {
  lateralPID.reset();
  angularPID.reset();
  lateralSmallExit.reset();
  lateralLargeExit.reset();
  angularSmallExit.reset();
  angularLargeExit.reset();
  prevLateralOutput = 0;
  prevAngularOutput = 0;
  distanceTraveled = 0;
}

float Actor::getDistanceTraveled() { return distanceTraveled; }

void Actor::setBrakeMode(pros::motor_brake_mode_e mode) {
  left_motors.set_brake_mode(mode);
  right_motors.set_brake_mode(mode);
}

void Actor::stop() {
  left_motors.move(0);
  right_motors.move(0);
}

// ========== Turn Functions ==========

void Actor::turnByDegrees(float delta_degrees, TurnToHeadingParams params,
                          int timeout) {
  float startHeading = odom.get_theta_degrees();
  float targetHeading = startHeading + delta_degrees;
  turnToHeading(targetHeading, params, timeout);
}

void Actor::turnToHeading(float target_degrees, TurnToHeadingParams params,
                          int timeout) {
  pros::lcd::print(2, "Turning to %.2f deg", target_degrees);

  // Reset state
  angularPID.reset();
  angularSmallExit.reset();
  angularLargeExit.reset();
  distanceTraveled = 0;

  float startHeading = odom.get_theta_degrees();
  int startTime = pros::millis();
  const int DT_MS = 10;

  while (true) {
    // Check timeout
    if (timeout > 0 && (pros::millis() - startTime) > timeout) break;

    float currentHeading = odom.get_theta_degrees();
    float error = angleError(target_degrees, currentHeading);

    // Track distance traveled (in degrees)
    distanceTraveled = std::abs(currentHeading - startHeading);

    // Calculate PID output
    float output = angularPID.update(error);

    // Apply slew rate limiting
    output = slew(output, prevAngularOutput, angularSettings.slew);
    prevAngularOutput = output;

    // Clamp to speed limits
    output = clampValue(output, -params.maxSpeed, params.maxSpeed);

    // Handle minimum speed for motion chaining
    if (params.minSpeed != 0 && std::abs(output) < params.minSpeed &&
        std::abs(error) > 0.5f) {
      output = (output > 0 ? params.minSpeed : -params.minSpeed);
    }

    // Send to motors (positive turn = clockwise = left forward, right backward)
    tank(output, -output);

    // Check exit conditions
    bool smallExitMet = angularSmallExit.update(std::abs(error));
    bool largeExitMet = angularLargeExit.update(std::abs(error));

    // Motion chaining exit (with minSpeed set)
    if (params.minSpeed != 0 && params.earlyExitRange != 0) {
      if (std::abs(error) < params.earlyExitRange) break;
    }
    // Standard exit (minSpeed = 0)
    else if (params.minSpeed == 0 && (smallExitMet || largeExitMet)) {
      break;
    }

    pros::delay(DT_MS);
  }

  // Only stop if not motion chaining
  if (params.minSpeed == 0) {
    stop();
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  }
}

void Actor::turnToPoint(float x, float y, TurnToPointParams params,
                        int timeout) {
  Pose pose = getPose();

  // Calculate target heading
  float dx = x - pose.x;
  float dy = y - pose.y;
  float targetHeading = 90.0f - std::atan2(dy, dx) * 180.0f / M_PI;

  // Reverse heading if facing backwards
  if (!params.forwards) {
    targetHeading += 180.0f;
  }

  // Normalize to 0-360
  targetHeading = std::fmod(targetHeading, 360.0f);
  if (targetHeading < 0) targetHeading += 360.0f;

  TurnToHeadingParams turnParams;
  turnParams.maxSpeed = params.maxSpeed;
  turnParams.minSpeed = params.minSpeed;
  turnParams.earlyExitRange = params.earlyExitRange;

  turnToHeading(targetHeading, turnParams, timeout);
}

// ========== Swing Turn Functions ==========

void Actor::swingToHeading(float target_degrees, DriveSide lockedSide,
                           SwingToHeadingParams params, int timeout) {
  pros::lcd::print(2, "Swing to %.2f deg (%s locked)", target_degrees,
                   lockedSide == DriveSide::LEFT ? "L" : "R");

  // Reset state
  angularPID.reset();
  angularSmallExit.reset();
  angularLargeExit.reset();
  distanceTraveled = 0;

  float startHeading = odom.get_theta_degrees();
  int startTime = pros::millis();
  const int DT_MS = 10;

  while (true) {
    // Check timeout
    if (timeout > 0 && (pros::millis() - startTime) > timeout) break;

    float currentHeading = odom.get_theta_degrees();
    float error = angleError(target_degrees, currentHeading);

    // Track distance traveled (in degrees)
    distanceTraveled = std::abs(currentHeading - startHeading);

    // Calculate PID output
    float output = angularPID.update(error);

    // Apply slew rate limiting
    output = slew(output, prevAngularOutput, angularSettings.slew);
    prevAngularOutput = output;

    // Clamp to speed limits
    output = clampValue(output, -params.maxSpeed, params.maxSpeed);

    // Handle minimum speed for motion chaining
    if (params.minSpeed != 0 && std::abs(output) < params.minSpeed &&
        std::abs(error) > 0.5f) {
      output = (output > 0 ? params.minSpeed : -params.minSpeed);
    }

    // Swing turn: only move one side of the drivetrain
    // When LEFT is locked: only right motors move
    //   - Positive output (turn CW) -> right motors forward
    //   - Negative output (turn CCW) -> right motors backward
    // When RIGHT is locked: only left motors move
    //   - Positive output (turn CW) -> left motors backward
    //   - Negative output (turn CCW) -> left motors forward
    if (lockedSide == DriveSide::LEFT) {
      // Left side locked (stationary), right side moves
      // Positive error = need to turn CW = right wheels forward
      tank(0, -output);
    } else {
      // Right side locked (stationary), left side moves
      // Positive error = need to turn CW = left wheels forward
      tank(output, 0);
    }

    // Check exit conditions
    bool smallExitMet = angularSmallExit.update(std::abs(error));
    bool largeExitMet = angularLargeExit.update(std::abs(error));

    // Motion chaining exit (with minSpeed set)
    if (params.minSpeed != 0 && params.earlyExitRange != 0) {
      if (std::abs(error) < params.earlyExitRange) break;
    }
    // Standard exit (minSpeed = 0)
    else if (params.minSpeed == 0 && (smallExitMet || largeExitMet)) {
      break;
    }

    pros::delay(DT_MS);
  }

  // Only stop if not motion chaining
  if (params.minSpeed == 0) {
    stop();
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  }
}

void Actor::swingToPoint(float x, float y, DriveSide lockedSide,
                         SwingToPointParams params, int timeout) {
  Pose pose = getPose();

  // Calculate target heading to face the point
  float dx = x - pose.x;
  float dy = y - pose.y;
  float targetHeading = 90.0f - std::atan2(dy, dx) * 180.0f / M_PI;

  // Reverse heading if facing backwards
  if (!params.forwards) {
    targetHeading += 180.0f;
  }

  // Normalize to 0-360
  targetHeading = std::fmod(targetHeading, 360.0f);
  if (targetHeading < 0) targetHeading += 360.0f;

  // Convert to SwingToHeadingParams and call swingToHeading
  SwingToHeadingParams swingParams;
  swingParams.maxSpeed = params.maxSpeed;
  swingParams.minSpeed = params.minSpeed;
  swingParams.earlyExitRange = params.earlyExitRange;

  swingToHeading(targetHeading, lockedSide, swingParams, timeout);
}

// ========== Drive Functions ==========

void Actor::driveStraight(float distance_inches, MoveToPointParams params,
                          int timeout) {
  pros::lcd::print(2, "Driving %.2f in", distance_inches);

  // Reset state
  lateralPID.reset();
  angularPID.reset();
  lateralSmallExit.reset();
  lateralLargeExit.reset();
  distanceTraveled = 0;

  Eigen::Vector2d startXY = odom.get_xy_inches();
  float startHeading = odom.get_theta_degrees();
  int startTime = pros::millis();
  const int DT_MS = 10;

  // Handle direction
  float direction = params.forwards ? 1.0f : -1.0f;
  float targetDistance = std::abs(distance_inches);

  while (true) {
    // Check timeout
    if (timeout > 0 && (pros::millis() - startTime) > timeout) break;

    Eigen::Vector2d currentXY = odom.get_xy_inches();
    float currentHeading = odom.get_theta_degrees();

    // Calculate traveled distance
    distanceTraveled = (currentXY - startXY).norm();
    float distanceError = targetDistance - distanceTraveled;

    // Calculate heading error (maintain starting heading)
    float headingError = angleError(startHeading, currentHeading);

    // Calculate PID outputs
    float lateralOutput = lateralPID.update(distanceError) * direction;
    float angularOutput = angularPID.update(headingError);

    // Apply slew rate limiting
    lateralOutput =
        slew(lateralOutput, prevLateralOutput, lateralSettings.slew);
    angularOutput =
        slew(angularOutput, prevAngularOutput, angularSettings.slew * 0.5f);
    prevLateralOutput = lateralOutput;
    prevAngularOutput = angularOutput;

    // Clamp to speed limits
    lateralOutput =
        clampValue(lateralOutput, -params.maxSpeed, params.maxSpeed);

    // Handle minimum speed for motion chaining
    if (params.minSpeed != 0 && std::abs(lateralOutput) < params.minSpeed &&
        distanceError > 0.5f) {
      lateralOutput = params.minSpeed * direction;
    }

    // Send to motors using arcade drive
    arcade(lateralOutput, angularOutput);

    // Check exit conditions
    bool smallExitMet = lateralSmallExit.update(std::abs(distanceError));
    bool largeExitMet = lateralLargeExit.update(std::abs(distanceError));

    // Motion chaining exit
    if (params.minSpeed != 0 && params.earlyExitRange != 0) {
      if (std::abs(distanceError) < params.earlyExitRange) break;
    }
    // Standard exit
    else if (params.minSpeed == 0 && (smallExitMet || largeExitMet)) {
      break;
    }

    pros::delay(DT_MS);
  }

  // Only stop if not motion chaining
  if (params.minSpeed == 0) {
    stop();
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  }
}

void Actor::driveTime(float time_ms, float speed_percent) {
  float startHeading = odom.get_theta_degrees();
  int startTime = pros::millis();
  const int DT_MS = 10;
  const float KP_HEADING = 0.85f;

  while ((pros::millis() - startTime) < time_ms) {
    float currentHeading = odom.get_theta_degrees();
    float headingError = angleError(startHeading, currentHeading);
    float headingCorrection = KP_HEADING * headingError;

    float left = speed_percent + headingCorrection;
    float right = speed_percent - headingCorrection;

    left = clampValue(left, -std::abs(speed_percent), std::abs(speed_percent));
    right =
        clampValue(right, -std::abs(speed_percent), std::abs(speed_percent));

    tank(left, right);
    pros::delay(DT_MS);
  }

  stop();
}

// ========== Point-to-Point Navigation ==========

void Actor::moveToPoint(float x, float y, MoveToPointParams params,
                        int timeout) {
  pros::lcd::print(2, "Moving to (%.1f, %.1f)", x, y);

  // Get current pose
  Pose pose = getPose();

  // Calculate distance and angle to target
  float dx = x - pose.x;
  float dy = y - pose.y;
  float distance = std::sqrt(dx * dx + dy * dy);

  // Calculate target heading (robot heading convention: 0 = forward, CW
  // positive)
  float targetHeading = 90.0f - std::atan2(dy, dx) * 180.0f / M_PI;

  // Reverse heading if driving backwards
  if (!params.forwards) {
    targetHeading += 180.0f;
  }

  // Normalize target heading to 0-360
  targetHeading = std::fmod(targetHeading, 360.0f);
  if (targetHeading < 0) targetHeading += 360.0f;

  // Split timeout between turn and drive (roughly 30% turn, 70% drive)
  int turnTimeout = timeout * 0.3f;
  int driveTimeout = timeout * 0.7f;

  // Step 1: Turn to face the target point
  TurnToHeadingParams turnParams;
  turnParams.maxSpeed = params.maxSpeed;
  turnParams.minSpeed = params.minSpeed;
  turnParams.earlyExitRange =
      params.earlyExitRange > 0 ? 5.0f : 0;  // Use 5 deg early exit if chaining
  turnToHeading(targetHeading, turnParams, turnTimeout);

  // Step 2: Drive straight to the target
  // Adjust distance based on how far we've already moved during the turn
  Pose newPose = getPose();
  float newDx = x - newPose.x;
  float newDy = y - newPose.y;
  float remainingDistance = std::sqrt(newDx * newDx + newDy * newDy);

  // Drive the remaining distance (negative if backwards)
  float driveDistance =
      params.forwards ? remainingDistance : -remainingDistance;

  MoveToPointParams driveParams;
  driveParams.forwards = params.forwards;
  driveParams.maxSpeed = params.maxSpeed;
  driveParams.minSpeed = params.minSpeed;
  driveParams.earlyExitRange = params.earlyExitRange;
  driveStraight(driveDistance, driveParams, driveTimeout);
}

// ========== Boomerang Controller (Move to Pose) ==========

void Actor::moveToPose(float x, float y, float theta, MoveToPoseParams params,
                       int timeout) {
  pros::lcd::print(2, "Moving to pose (%.1f, %.1f, %.1f)", x, y, theta);

  // Reset state
  lateralPID.reset();
  angularPID.reset();
  lateralSmallExit.reset();
  lateralLargeExit.reset();
  distanceTraveled = 0;

  Pose startPose = getPose();
  int startTime = pros::millis();
  const int DT_MS = 10;

  while (true) {
    // Check timeout
    if (timeout > 0 && (pros::millis() - startTime) > timeout) break;

    Pose pose = getPose();

    // Calculate distance to target
    float dx = x - pose.x;
    float dy = y - pose.y;
    float distanceToTarget = std::sqrt(dx * dx + dy * dy);

    // Track distance traveled
    float startDx = pose.x - startPose.x;
    float startDy = pose.y - startPose.y;
    distanceTraveled = std::sqrt(startDx * startDx + startDy * startDy);

    // Calculate carrot point (boomerang algorithm)
    // The carrot leads the target by a distance proportional to remaining
    // distance
    float carrotX, carrotY;

    if (distanceToTarget > 1.0f) {
      // Carrot point is placed between current position and target,
      // offset towards the final heading
      float targetHeadingRad = theta * M_PI / 180.0f;
      float carrotDistance = distanceToTarget * params.lead;

      // Place carrot in direction of target heading, behind the target
      carrotX = x - carrotDistance * std::sin(targetHeadingRad);
      carrotY = y - carrotDistance * std::cos(targetHeadingRad);
    } else {
      // Close to target, aim directly at final position
      carrotX = x;
      carrotY = y;
    }

    // Calculate heading to carrot point
    float carrotDx = carrotX - pose.x;
    float carrotDy = carrotY - pose.y;
    float targetHeading =
        90.0f - std::atan2(carrotDy, carrotDx) * 180.0f / M_PI;

    // Reverse if driving backwards
    if (!params.forwards) {
      targetHeading += 180.0f;
    }

    // Normalize target heading
    targetHeading = std::fmod(targetHeading, 360.0f);
    if (targetHeading < 0) targetHeading += 360.0f;

    float headingError = angleError(targetHeading, pose.theta);

    // Direction multiplier
    float direction = params.forwards ? 1.0f : -1.0f;

    // Reduce speed when not facing target
    float headingFactor = std::cos(headingError * M_PI / 180.0f);
    if (!params.forwards) headingFactor = -headingFactor;

    // Calculate PID outputs
    float lateralOutput = lateralPID.update(distanceToTarget) * headingFactor;
    float angularOutput = angularPID.update(headingError);

    // Apply slew rate limiting
    lateralOutput =
        slew(lateralOutput, prevLateralOutput, lateralSettings.slew);
    angularOutput =
        slew(angularOutput, prevAngularOutput, angularSettings.slew);
    prevLateralOutput = lateralOutput;
    prevAngularOutput = angularOutput;

    // Clamp to speed limits
    lateralOutput =
        clampValue(lateralOutput, -params.maxSpeed, params.maxSpeed);

    // Handle minimum speed for motion chaining
    if (params.minSpeed != 0 && std::abs(lateralOutput) < params.minSpeed &&
        distanceToTarget > 1.0f) {
      lateralOutput = params.minSpeed * direction;
    }

    // Send to motors
    arcade(lateralOutput, angularOutput);

    // Check exit conditions
    bool smallExitMet = lateralSmallExit.update(distanceToTarget);
    bool largeExitMet = lateralLargeExit.update(distanceToTarget);

    // Motion chaining exit
    if (params.minSpeed != 0 && params.earlyExitRange != 0) {
      if (distanceToTarget < params.earlyExitRange) break;
    }
    // Standard exit
    else if (params.minSpeed == 0 && (smallExitMet || largeExitMet)) {
      break;
    }

    pros::delay(DT_MS);
  }

  // Final heading correction if not motion chaining
  if (params.minSpeed == 0) {
    TurnToHeadingParams turnParams;
    turnParams.maxSpeed = params.maxSpeed * 0.7f;
    turnToHeading(theta, turnParams, 1500);

    stop();
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  }
}

void Actor::waitUntil(float dist) {
  while (distanceTraveled < dist) {
    pros::delay(10);
  }
}

void Actor::wiggle(float speed_start, float speed_end, float distance_inches,
                   float wiggle_degrees, float wiggle_period,
                   float timeout_milliseconds) {
  // Get starting pose
  Eigen::Vector2d startXY = odom.get_xy_inches();
  float startHeading = odom.get_theta_degrees();
  float traveled = 0;
  int startTime = pros::millis();
  const int DT_MS = 10;
  float direction = (speed_start >= 0) ? 1.0f : -1.0f;

  while (traveled < std::abs(distance_inches) &&
         (pros::millis() - startTime) < timeout_milliseconds) {
    float elapsed = (pros::millis() - startTime) / 1000.0f;
    // Calculate target heading: startHeading + sin wave
    float wiggle =
        std::sin(2 * M_PI * elapsed / wiggle_period) * wiggle_degrees;
    float targetHeading = startHeading + wiggle;
    float currentHeading = odom.get_theta_degrees();
    float headingError = angleError(targetHeading, currentHeading);

    // Simple P controller for heading
    float turn = 2.0f * headingError;  // 2.0 is a reasonable P gain for heading
    float throttle = speed_start + (speed_end - speed_start) *
                                       (traveled / std::abs(distance_inches));
    arcade(throttle, turn);

    // Update traveled distance
    Eigen::Vector2d currentXY = odom.get_xy_inches();
    traveled = (currentXY - startXY).norm();
    pros::delay(DT_MS);
  }
  stop();
}
