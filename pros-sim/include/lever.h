#ifndef LEVER_H
#define LEVER_H

#include <mutex>
#include <vector>
#include <initializer_list>
#include "api.h"

// this file is used to make a general class for an actuating lever mechanism controlled by pistons moving in unison

class Lever {
private:
  pros::Mutex mtx;
  std::vector<pros::adi::Pneumatics*> pistons;
  bool is_extended;

  void extend_impl();
  void retract_impl();

public:
  // Constructor accepting an initializer list of piston pointers
  Lever(std::initializer_list<pros::adi::Pneumatics*> piston_list);

  // Constructor accepting a vector of piston pointers
  Lever(std::vector<pros::adi::Pneumatics*> piston_list);

  void extend();
  void retract();
  void toggle();
  bool get_state() const;

  // Get the number of pistons
  size_t piston_count() const;
};

#endif // LEVER_H