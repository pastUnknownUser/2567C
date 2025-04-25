#pragma once

#include "EZ-Template/PID.hpp"
#include "EZ-Template/piston.hpp" // IWYU pragma: keep
#include "api.h" // IWYU pragma: keep
#include "pros/rotation.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(-3);
inline pros::Motor lbm1(14);
inline pros::Motor lbm2(2);
inline pros::Optical colorSensor(10);
inline pros::Rotation lb(13);
inline ez::PID Lady{1.5, 0.003, 4, 100, "Lady"};
inline ez::Piston lDoinker('H');
inline ez::Piston mogoClamp('B');
inline ez::Piston rDoinker('A');
inline ez::Piston IntakeLift('G');
