#include "distance_sensor.h"

static constexpr double MM_TO_INCHES = 1.0 / 25.4;
static constexpr int MAX_VALID_MM = 2000;
static constexpr int MIN_CONFIDENCE = 15;

DistanceSensor::DistanceSensor(int port) : sensor(port) {}

double DistanceSensor::get_distance_inches() {
  return sensor.get_distance() * MM_TO_INCHES;
}

int DistanceSensor::get_distance_mm() {
  return sensor.get_distance();
}

int DistanceSensor::get_confidence() {
  return sensor.get_confidence();
}

bool DistanceSensor::is_valid_reading() {
  int dist = sensor.get_distance();
  int conf = sensor.get_confidence();
  return dist < MAX_VALID_MM && dist != PROS_ERR && conf > MIN_CONFIDENCE;
}
