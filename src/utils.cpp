#include "EZ-Template/util.hpp" // IWYU pragma: keep
#include "autons.hpp"
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "pros/colors.h" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "global.h"
#include <iostream>

void live_Data() {
  while (true) {
    std::cout << "Heading: " << chassis.drive_imu_get() << std::endl;
    std::cout << "Hue: " << intakeStopper.get_hue() << std::endl;
    pros::delay(100);
  }
}

void ringDirect() {
  intakeToggleControl++;
  stopIntake();
  pros::delay(100);
  intake.move_relative(270, 100);
  intake2.move_relative(-270, 100);
  pros::delay(1000);
  std::cout << "adkjhafklsjhdflkjsdhf" << std::endl;
}

void autoRingDirect() {
  int ringCount = 0;
  intakeStopper.set_led_pwm(100);
  while (true) {
    if (intakeStopper.get_hue() < 15) {
      stopIntake();
      pros::delay(100);
      intake.move_relative(-270, 100);
      intake.move_relative(-270, 100);
      ringCount++;
    }
  }
}