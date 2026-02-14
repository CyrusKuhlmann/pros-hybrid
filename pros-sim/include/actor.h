#pragma once

#include "Eigen/Dense"
#include "api.h"
#include "odom.h"
#include "pid.h"

/**
 * @brief Controller settings for PID motion chaining
 *
 * Provides all the parameters needed for PID control with exit conditions
 * and slew rate limiting for smooth motion chaining.
 */
struct MotionControllerSettings {
  float kP;                 // Proportional gain
  float kI;                 // Integral gain
  float kD;                 // Derivative gain
  float windupRange;        // Integral anti-windup range
  float smallError;         // Small error range for exit condition
  float smallErrorTimeout;  // Timeout for small error exit (ms)
  float largeError;         // Large error range for exit condition
  float largeErrorTimeout;  // Timeout for large error exit (ms)
  float slew;               // Maximum slew rate (% per update)
};

/**
 * @brief Parameters for move-to-point motion
 */
struct MoveToPointParams {
  bool forwards = true;      // Drive forwards or backwards
  float maxSpeed = 100;      // Maximum speed (0-100%)
  float minSpeed = 0;        // Minimum speed for motion chaining
  float earlyExitRange = 0;  // Early exit distance (inches)
};

/**
 * @brief Parameters for turn-to-heading motion
 */
struct TurnToHeadingParams {
  float maxSpeed = 100;      // Maximum speed (0-100%)
  float minSpeed = 0;        // Minimum speed for motion chaining
  float earlyExitRange = 0;  // Early exit angle (degrees)
};

/**
 * @brief Parameters for turn-to-point motion
 */
struct TurnToPointParams {
  bool forwards = true;      // Face point with front or back
  float maxSpeed = 100;      // Maximum speed (0-100%)
  float minSpeed = 0;        // Minimum speed for motion chaining
  float earlyExitRange = 0;  // Early exit angle (degrees)
};

/**
 * @brief Parameters for move-to-pose motion (boomerang-style)
 */
struct MoveToPoseParams {
  bool forwards = true;      // Drive forwards or backwards
  float lead = 0.6;          // Carrot point lead (0-1, higher = curvier)
  float maxSpeed = 100;      // Maximum speed (0-100%)
  float minSpeed = 0;        // Minimum speed for motion chaining
  float earlyExitRange = 0;  // Early exit distance (inches)
};

/**
 * @brief Enum for specifying which side of the drivetrain to lock during swing
 * turns
 */
enum class DriveSide {
  LEFT,  // Lock left side, only right motors move
  RIGHT  // Lock right side, only left motors move
};

/**
 * @brief Parameters for swing-to-heading motion
 */
struct SwingToHeadingParams {
  float maxSpeed = 100;      // Maximum speed (0-100%)
  float minSpeed = 0;        // Minimum speed for motion chaining
  float earlyExitRange = 0;  // Early exit angle (degrees)
};

/**
 * @brief Parameters for swing-to-point motion
 */
struct SwingToPointParams {
  bool forwards = true;      // Face point with front or back
  float maxSpeed = 100;      // Maximum speed (0-100%)
  float minSpeed = 0;        // Minimum speed for motion chaining
  float earlyExitRange = 0;  // Early exit angle (degrees)
};

/**
 * @brief Actor class using PID controllers with custom odometry
 *
 * This class provides motion control using custom PID implementation
 * while leveraging the existing custom odometry system for position tracking.
 * Supports motion chaining through minSpeed and earlyExitRange parameters.
 */
class Actor {
 private:
  Odom& odom;
  pros::MotorGroup& left_motors;
  pros::MotorGroup& right_motors;

  // PID controllers
  PID lateralPID;
  PID angularPID;

  // Controller settings
  MotionControllerSettings lateralSettings;
  MotionControllerSettings angularSettings;

  // Exit conditions
  ExitCondition lateralSmallExit;
  ExitCondition lateralLargeExit;
  ExitCondition angularSmallExit;
  ExitCondition angularLargeExit;

  // State tracking for motion chaining
  float prevLateralOutput = 0;
  float prevAngularOutput = 0;
  float distanceTraveled = 0;

  // Helper functions
  float slew(float target, float current, float maxChange);
  void tank(float left, float right);
  void arcade(float throttle, float turn);
  float angleError(float target, float current);

 public:
  /**
   * @brief Construct Actor with custom PID settings
   */
  Actor(Odom& odom_ref, pros::MotorGroup& left_motors_ref,
        pros::MotorGroup& right_motors_ref,
        MotionControllerSettings lateral_settings,
        MotionControllerSettings angular_settings);

  /**
   * @brief Turn the robot by a relative angle
   * @param delta_degrees Angle to turn (positive = clockwise)
   * @param params Turn parameters for motion chaining
   * @param timeout Maximum time for movement (ms), 0 = no timeout
   */
  void turnByDegrees(float delta_degrees, TurnToHeadingParams params = {},
                     int timeout = 5000);

  /**
   * @brief Turn to face an absolute heading
   * @param target_degrees Target heading (0 = starting orientation)
   * @param params Turn parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void turnToHeading(float target_degrees, TurnToHeadingParams params = {},
                     int timeout = 5000);

  /**
   * @brief Turn to face a specific point
   * @param x Target x coordinate (inches)
   * @param y Target y coordinate (inches)
   * @param params Turn parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void turnToPoint(float x, float y, TurnToPointParams params = {},
                   int timeout = 5000);

  /**
   * @brief Swing turn to face a heading using only one side of the drivetrain
   * @param target_degrees Target heading (0 = starting orientation)
   * @param lockedSide Which side of the drivetrain to lock (LEFT or RIGHT)
   * @param params Swing parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void swingToHeading(float target_degrees, DriveSide lockedSide,
                      SwingToHeadingParams params = {}, int timeout = 5000);

  /**
   * @brief Swing turn to face a point using only one side of the drivetrain
   * @param x Target x coordinate (inches)
   * @param y Target y coordinate (inches)
   * @param lockedSide Which side of the drivetrain to lock (LEFT or RIGHT)
   * @param params Swing parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void swingToPoint(float x, float y, DriveSide lockedSide,
                    SwingToPointParams params = {}, int timeout = 5000);

  /**
   * @brief Drive straight for a specified distance
   * @param distance_inches Distance to travel (positive = forward)
   * @param params Move parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void driveStraight(float distance_inches, MoveToPointParams params = {},
                     int timeout = 5000);

  /**
   * @brief Move to a specific point on the field
   * @param x Target x coordinate (inches)
   * @param y Target y coordinate (inches)
   * @param params Move parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void moveToPoint(float x, float y, MoveToPointParams params = {},
                   int timeout = 5000);

  /**
   * @brief Move to a pose with specified heading (boomerang controller)
   * @param x Target x coordinate (inches)
   * @param y Target y coordinate (inches)
   * @param theta Target heading (degrees)
   * @param params Move-to-pose parameters for motion chaining
   * @param timeout Maximum time for movement (ms)
   */
  void moveToPose(float x, float y, float theta, MoveToPoseParams params = {},
                  int timeout = 5000);

  /**
   * @brief Drive at a fixed speed for a specified time
   * @param time_ms Duration (milliseconds)
   * @param speed_percent Speed (-100 to 100)
   */
  void driveTime(float time_ms, float speed_percent);

  /**
   * @brief Drive straight while wiggling heading
   * @param speed Speed to drive (-100 to 100)
   * @param distance_inches Distance to travel (inches)
   * @param wiggle_degrees Amplitude of heading wiggle (degrees)
   */
  void wiggle(float speed_start, float speed_end, float distance_inches,
              float wiggle_degrees, float wiggle_period,
              float timeout_milliseconds);

  /**
   * @brief Get the current pose from odometry
   * @return Pose Current robot pose
   */
  Pose getPose();

  /**
   * @brief Reset PID controllers and motion state
   */
  void resetMotionState();

  /**
   * @brief Wait until robot has traveled specified distance
   * @param dist Distance threshold (inches for linear, degrees for angular)
   */
  void waitUntil(float dist);

  /**
   * @brief Get distance traveled during current motion
   * @return Distance in inches
   */
  float getDistanceTraveled();

  /**
   * @brief Set brake mode for drivetrain
   * @param mode Brake mode to set
   */
  void setBrakeMode(pros::motor_brake_mode_e mode);

  /**
   * @brief Stop the drivetrain
   */
  void stop();
};
