#pragma once

#include <cmath>

#include "Eigen/Dense"
#include "api.h"
#include "pod.h"

// Global rotation and IMU objects
extern pros::Rotation lateral_rot;
extern pros::Rotation forward_rot;
extern pros::Imu imu;

const double TRACKING_WHEEL_DIAMETER_INCHES = 1.9965;
const double FORWARD_CORRECTION_CLOCK = 0.122668;          // inches per radian
const double FORWARD_CORRECTION_COUNTERCLOCK = 0.130898;  // inches per radian
const double LATERAL_CORRECTION_CLOCK = 1.10856;          // inches per radian
const double LATERAL_CORRECTION_COUNTERCLOCK = 1.3381;   // inches per radian
const double IMU_CORRECTION_FACTOR =
    1.0059 * (360.0 / 359.0) * (360.0/361.11393);  // multiplier to correct IMU drift

class Odom {
 private:
  OdomPod forward_pod;
  OdomPod lateral_pod;
  // Robot orientation
  double delta_theta_degrees;
  double theta_degrees;
  double prev_theta_degrees;
  double average_theta_degrees;

  // Robot position
  Eigen::Matrix<double, 2, 1> xy;
  Eigen::Matrix<double, 2, 1> prev_xy;
  double rad_to_deg(double rad);
  double deg_to_rad(double deg);
  void update_raw_values();
  void update_xy();
  void debug(int i);

 public:
  // Constructor
  Odom()
      : forward_pod(TRACKING_WHEEL_DIAMETER_INCHES, FORWARD_CORRECTION_CLOCK,
                    FORWARD_CORRECTION_COUNTERCLOCK),
        lateral_pod(TRACKING_WHEEL_DIAMETER_INCHES, LATERAL_CORRECTION_CLOCK,
                    LATERAL_CORRECTION_COUNTERCLOCK),
        delta_theta_degrees(0.0),
        theta_degrees(0.0),
        prev_theta_degrees(0.0),
        average_theta_degrees(0.0),
        xy(Eigen::Matrix<double, 2, 1>::Zero()),
        prev_xy(Eigen::Matrix<double, 2, 1>::Zero()) {}

  // Methods
  void odom_task_fn();
  Eigen::Matrix<double, 2, 1> get_xy_inches();
  double get_theta_degrees();
  double distance_to_point_inches(Eigen::Matrix<double, 2, 1> target_xy);
  double angle_to_heading_degrees(double target_degrees);
  double angle_to_point_degrees(Eigen::Matrix<double, 2, 1> target_xy);
  void manual_set_xy(double x_inches, double y_inches);
};
