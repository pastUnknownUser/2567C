#include "EZ-Template/PID.hpp" // IWYU pragma: keep
#include "EZ-Template/util.hpp" // IWYU pragma: keep
#include "autons.hpp"
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "pros/colors.h" // IWYU pragma: keep
#include "pros/colors.hpp"
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "global.h"
#include <iostream>

void live_Data() {
  while (true) {
    std::cout << "Heading: " << chassis.drive_imu_get() << std::endl;
    std::cout << "Hue: " << colorSensor.get_hue() << std::endl;
    std::cout << "POS: " << lb.get_position() << std::endl;
    pros::delay(100);
  }
}

void ringDirect() {
  intakeToggleControl++;
  stopIntake();
  pros::delay(100);

  pros::delay(1000);
  std::cout << "adkjhafklsjhdflkjsdhf" << std::endl;
}

void autoRingDirect() {
  int ringCount = 0;
  colorSensor.set_led_pwm(100);
  while (true) {
    if (colorSensor.get_hue() < 15) {
      stopIntake();
      pros::delay(100);
      
      ringCount++;
    }
  }
}

void colorSort() {
  while (true) {
    runIntake();
    if (color == 0) { // blue
      if (colorSensor.get_hue()) {
        stopIntake();
        pros::delay(100);
      }
    }
    if (color == 1) { // red
      if (colorSensor.get_hue()) { 
        stopIntake();
        pros::delay(100);
      }
    }
    pros::delay(20);
  }
}
