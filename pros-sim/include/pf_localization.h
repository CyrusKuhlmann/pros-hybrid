#pragma once

#include <atomic>

#include "Eigen/Dense"
#include "api.h"
#include "distance_sensor.h"
#include "irobot_state.h"
#include "odom.h"
#include "particle_filter.h"

// ---- Configuration (adjust to your robot) ----
const int PF_LEFT_DISTANCE_PORT = 5;   // V5 port for left distance sensor
const int PF_RIGHT_DISTANCE_PORT = 6;  // V5 port for right distance sensor
const int PF_NUM_PARTICLES = 2000;
const float PF_INITIAL_SPREAD = 5.0f;  // inches – gaussian spread at init
const int PF_UPDATE_INTERVAL_MS = 1;  // how often the PF task runs

class PFLocalization : public IRobotState {
private:
  Odom& odom;
  DistanceSensor left_distance;
  DistanceSensor right_distance;
  ParticleFilter pf;
  std::atomic<bool> initialized;

  // Track previous odom position to compute per-cycle deltas
  Eigen::Matrix<double, 2, 1> prev_odom_xy;

  void update();

public:
  PFLocalization(Odom& odom);

  /// Initialize particles around a known starting position.
  void initialize(float start_x, float start_y,
    float spread = PF_INITIAL_SPREAD);

  /// Get the particle filter's best position estimate (inches).
  Eigen::Vector2f get_estimate() const;

  /// IRobotState interface – returns PF estimate as double vector.
  Eigen::Matrix<double, 2, 1> get_xy_inches() override;

  /// IRobotState interface – delegates to internal odom heading.
  double get_theta_degrees() override;

  /// Blocking task loop – pass to pros::Task.
  void task_fn();
};
