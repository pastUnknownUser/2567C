#include "main.h"
#include "EZ-Template/PID.hpp"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/auton_selector.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "liblvgl/font/lv_symbol_def.h" // IWYU pragma: keep
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp" // IWYU pragma: keep
#include "utils.hpp"
#include "global.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-15, 6,-11},     // Left Chassis Ports (negative port will reverse it!)
    {-18, 17, 20},  // Right Chassis Ports (negative port will reverse it!)

    3,      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("red right rush", red_right_rush),
      Auton("red right wp", red_right_win_point),
      Auton("red left wp", red_left_win_point),
      Auton("blue right wp", blue_right_win_point),
      Auton("blue left rush", blue_left_rush),
      Auton("blue left wp", blue_left_win_point),
      Auton("SKILLS", skills),
      Auton("red left four", red_left_four),
      Auton("blue right four", blue_right_four),
      Auton("test", test),
    
     
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  Lady.i_reset_toggle(false);
  //pros::Task heading(live_heading);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  pros::Task data(live_Data);
  //intakeStopper.set_led_pwm(100);
  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

  chassis.drive_brake_set(driver_preference_brake);

  Lady.exit_condition_set(100, 3, 400, 7, 100, 200);
  
  lb.reset();

  int ladyvar = 2;

  int frontEnable = 0;

  lbm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  bool PIDenable = true;

  pros::Task asdf(colorSort);

  while (true) {
    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    if ((colorSorterControl % 2) == 0) {
      //asdf.suspend();
      if (master.get_digital(DIGITAL_R1)) {
        frontstage.move_voltage(12000);
        hooks.move_voltage(12000);
      } else if (master.get_digital(DIGITAL_R2)) {
        frontstage.move_voltage(-12000);
        hooks.move_voltage(-12000);
      } else {
        frontstage.move_voltage(0);
        hooks.move_voltage(0);
      }
    }

    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      ladyvar++;
      PIDenable = true;
    }

    if (PIDenable == true) {
      if ((ladyvar % 3) == 2) {
        Lady.target_set(116);
      }
  
      if ((ladyvar % 3) == 0) {
        Lady.target_set(137);
      }
  
      if((ladyvar % 3) == 1) {
        Lady.target_set(266);
      }
    }

    if (master.get_digital(DIGITAL_L1)) {
      PIDenable = false;
    } 
    
    if (master.get_digital(DIGITAL_L2)) {
      PIDenable = false;
    } 

    if (PIDenable == false) {
      if (master.get_digital(DIGITAL_L1)) {
        lbm.move_voltage(12000);
      } else if (master.get_digital(DIGITAL_L2)) {
        lbm.move_voltage(-12000);
      } else {
        lbm.move_voltage(0);
        lbm.brake();
      }
    }

    if (master.get_digital_new_press(DIGITAL_UP)) {
      frontEnable++;
    }

    if ((frontEnable % 2) == 1) {
      frontstage.move_voltage(12000);
      if (master.get_digital(DIGITAL_R1)) {
        hooks.move(12000);
      } else {
        hooks.move_voltage(0);
      }
    }

    if((frontEnable % 2) == 0) {
      if (master.get_digital(DIGITAL_R1)) {
        frontstage.move_voltage(12000);
        hooks.move_voltage(12000);
      } else if (master.get_digital(DIGITAL_R2)) {
        frontstage.move_voltage(-12000);
        hooks.move_voltage(-12000);
      } else {
        frontstage.move_voltage(0);
        hooks.move_voltage(0);
      }
    }

    mogoClamp.button_toggle(master.get_digital(DIGITAL_B));
    lDoinker.button_toggle(master.get_digital(DIGITAL_RIGHT));
    rDoinker.button_toggle(master.get_digital(DIGITAL_Y));
    
    if (PIDenable == true) {
      lbm.move(Lady.compute((lb.get_position())/100.0));
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}