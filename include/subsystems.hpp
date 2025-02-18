#pragma once

#include "EZ-Template/piston.hpp"
#include "api.h" // IWYU pragma: keep

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(11);
inline pros::Motor intake2(18);
inline ez::Piston sweepArm('H');
inline ez::Piston mogoClamp('A');
inline pros::Motor arm(13);
inline pros::Optical intakeStopper(19);
inline ez::Piston armPTO('B');
inline ez::Piston rushMech('D');