#include "pf_localization.h"

#include <cmath>

PFLocalization::PFLocalization(Odom& odom)
  : odom(odom),
  left_distance(PF_LEFT_DISTANCE_PORT),
  right_distance(PF_RIGHT_DISTANCE_PORT),
  pf(PF_NUM_PARTICLES),
  initialized{ false },
  prev_odom_xy(Eigen::Matrix<double, 2, 1>::Zero()) {
}

void PFLocalization::initialize(float start_x, float start_y, float spread) {
  pf.initialize(start_x, start_y, spread);
  prev_odom_xy = odom.get_xy_inches();
  initialized = true;
}

Eigen::Vector2f PFLocalization::get_estimate() const {
  return pf.estimate();
}

void PFLocalization::update() {
  if (!initialized) return;

  // Compute motion delta from odom since last cycle
  Eigen::Matrix<double, 2, 1> current_xy = odom.get_xy_inches();
  Eigen::Matrix<double, 2, 1> delta = current_xy - prev_odom_xy;
  prev_odom_xy = current_xy;

  pf.update_motion(static_cast<float>(delta(0)),
    static_cast<float>(delta(1)));


  float left_inches = static_cast<float>(left_distance.get_distance_inches());
  float right_inches = static_cast<float>(right_distance.get_distance_inches());
  float heading_rad = static_cast<float>(odom.get_theta_degrees() * M_PI / 180.0);
  if (left_inches < right_inches && left_distance.is_valid_reading()) {
    pf.update_sensor(left_inches, right_inches, heading_rad);
    pf.resample();
  }
  else if (right_inches < left_inches && right_distance.is_valid_reading()) {
    pf.update_sensor(left_inches, right_inches, heading_rad);
    pf.resample();
  }
}

Eigen::Matrix<double, 2, 1> PFLocalization::get_xy_inches() {
  Eigen::Vector2f est = pf.estimate();
  return Eigen::Matrix<double, 2, 1>(
    static_cast<double>(est.x()),
    static_cast<double>(est.y()));
}

double PFLocalization::get_theta_degrees() {
  return odom.get_theta_degrees();
}

void PFLocalization::task_fn() {
  while (true) {
    update();

    if (initialized) {
      Eigen::Vector2f est = pf.estimate();
      // pros::lcd::print(3, "PF: %.1f, %.1f", est.x(), est.y());
      pf.draw_particles();
    }

    pros::delay(PF_UPDATE_INTERVAL_MS);
  }
}
