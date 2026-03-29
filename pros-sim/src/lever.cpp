#include "lever.h"
// this file implements the Lever class methods

// Constructor accepting an initializer list of piston pointers
Lever::Lever(std::initializer_list<pros::adi::Pneumatics*> piston_list)
  : pistons(piston_list), is_extended(false) {
}

// Constructor accepting a vector of piston pointers
Lever::Lever(std::vector<pros::adi::Pneumatics*> piston_list)
  : pistons(std::move(piston_list)), is_extended(false) {
}

// Private unlocked implementations
void Lever::extend_impl() {
  for (auto* piston : pistons) {
    if (piston) {
      piston->set_value(true);
    }
  }
  is_extended = true;
}

void Lever::retract_impl() {
  for (auto* piston : pistons) {
    if (piston) {
      piston->set_value(false);
    }
  }
  is_extended = false;
}

// Method to extend the lever
void Lever::extend() {
  std::lock_guard<pros::Mutex> lock(mtx);
  extend_impl();
}

// Method to retract the lever
void Lever::retract() {
  std::lock_guard<pros::Mutex> lock(mtx);
  retract_impl();
}

// Method to toggle the lever state
void Lever::toggle() {
  std::lock_guard<pros::Mutex> lock(mtx);
  if (is_extended) {
    retract_impl();
  }
  else {
    extend_impl();
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