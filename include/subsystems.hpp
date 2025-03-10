#pragma once

#include "EZ-Template/PID.hpp"
#include "EZ-Template/piston.hpp" // IWYU pragma: keep
#include "api.h" // IWYU pragma: keep
#include "pros/rotation.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor frontstage(5);
inline pros::Motor hooks(-1);
inline pros::Motor lbm(14);
inline pros::Optical colorSensor(10);
inline pros::Rotation lb(9);
inline ez::PID Lady{1.5, 0.003, 4, 100, "Lady"};
inline ez::Piston lDoinker('H');
inline ez::Piston mogoClamp('F');
inline ez::Piston rDoinker('G');
inline ez::Piston IntakeLift('A');
