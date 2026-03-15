#pragma once

#include "api.h"

class DistanceSensor {
private:
  pros::Distance sensor;

public:
  DistanceSensor(int port);

  /// Get the distance reading converted to inches.
  double get_distance_inches();

  /// Get the raw distance reading in millimeters.
  int get_distance_mm();

  /// Get the confidence value (0–63, higher is better).
  int get_confidence();

  /// Returns true if the current reading is within valid range and confidence.
  bool is_valid_reading();
};
