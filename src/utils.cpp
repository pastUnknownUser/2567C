#include "EZ-Template/PID.hpp" // IWYU pragma: keep
#include "EZ-Template/util.hpp" // IWYU pragma: keep
#include "autons.hpp"
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "pros/colors.h" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "global.h"
#include <iostream>

void live_Data() {
  while (true) {
    //std::cout << "Heading: " << chassis.drive_imu_get() << std::endl;
    std::cout << "Hue: " << colorSensor.get_hue() << std::endl;
    //std::cout << "POS: " << lb.get_position() << std::endl;
    //std::cout << "Power: " << hooks.get_power() << std::endl;
    pros::delay(100);
  }
}


