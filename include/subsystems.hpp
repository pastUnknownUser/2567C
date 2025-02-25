#pragma once

#include "EZ-Template/piston.hpp" // IWYU pragma: keep
#include "api.h" // IWYU pragma: keep
#include "pros/rotation.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor frontstage(16);
inline pros::Motor hooks(-19);
inline pros::Motor lbm(15);
inline pros::Optical intakeStopper(10);
inline pros::Rotation lb(9);
