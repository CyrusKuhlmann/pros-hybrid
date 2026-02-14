#include "lever.h"
// this file implements the Lever class methods

// Constructor accepting an initializer list of piston pointers
Lever::Lever(std::initializer_list<pros::adi::Pneumatics*> piston_list)
    : pistons(piston_list), is_extended(false) {}

// Constructor accepting a vector of piston pointers
Lever::Lever(std::vector<pros::adi::Pneumatics*> piston_list)
    : pistons(std::move(piston_list)), is_extended(false) {}

// Method to extend the lever
void Lever::extend() {
  for (auto* piston : pistons) {
    if (piston) {
      piston->set_value(true);
    }
  }
  is_extended = true;
}

// Method to retract the lever
void Lever::retract() {
  for (auto* piston : pistons) {
    if (piston) {
      piston->set_value(false);
    }
  }
  is_extended = false;
}

// Method to toggle the lever state
void Lever::toggle() {
  if (is_extended) {
    retract();
  } else {
    extend();
  }
}

// Method to get the current state of the lever
bool Lever::get_state() const {
  return is_extended;
}

// Get the number of pistons
size_t Lever::piston_count() const {
  return pistons.size();
}