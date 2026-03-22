#pragma once

#include "Eigen/Dense"

class IRobotState {
public:
  virtual ~IRobotState() = default;
  virtual Eigen::Matrix<double, 2, 1> get_xy_inches() = 0;
  virtual double get_theta_degrees() = 0;
};
