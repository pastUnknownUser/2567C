#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/rtos.h" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "utils.hpp"
#include "global.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

int lbrd;

void fwrd(float distance, int speed) {
  chassis.pid_odom_set(distance, speed, true);
  chassis.pid_wait();
}

void rvs(float distance, int speed) {
  chassis.pid_odom_set(-distance, speed, true);
  chassis.pid_wait();
}

void fwrdUntil(float distance, int speed, float untilDistance, int untilSpeed) {
  chassis.pid_drive_set(distance, speed, true);
  chassis.pid_wait_until(untilDistance);
  chassis.pid_speed_max_set(untilSpeed);
  chassis.pid_wait();
}

void rvsUntil(float distance, int speed, float untilDistance, int untilSpeed) {
  chassis.pid_drive_set(-distance, speed, true);
  chassis.pid_wait_until(-untilDistance);
  chassis.pid_speed_max_set(untilSpeed);
  chassis.pid_wait();
}

void turn(float heading, int speed) {
  chassis.pid_turn_set(heading, speed, true);
  chassis.pid_wait();
}

void turnUntil(float heading, int speed, float untilAngle, int untilSpeed) {
  chassis.pid_turn_set(heading, speed, true);
  chassis.pid_wait_until(untilAngle);
  chassis.pid_speed_max_set(untilSpeed);
  chassis.pid_wait();
}

void runIntake(float voltage) {
  hooks.move_voltage(voltage); // intake out
  frontstage.move_voltage(voltage);
}

void revRunIntake(float voltage) {
  hooks.move_voltage(-voltage); // intake out
  frontstage.move_voltage(-voltage);
}

void stopIntake() {
  hooks.move_voltage(0);
  frontstage.move_voltage(0);
}

inline void set_lift(int input) {
  lbm.move(input);
}

void lbr() {
  Lady.exit_condition_set(100, 3, 400, 7, 100, 200);
  lb.reset();
  while (true) {
    Lady.target_set(lbrd);
    double output = Lady.compute(lb.get_position()/100.0);
    set_lift(output);
    pros::delay(ez::util::DELAY_TIME);
  }
  
}

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void solowinpointstate() {
  pros::Task ladybrown(lbr);
  fwrd(6, 110);
  lbrd = 310;
  pros::delay(800);
  rvs( 22, 127);
  rvs(14,65);
  lbrd = 100;
 pros::delay(150);
 mogoClamp.set(true);
 pros::delay(150);
 turn(125, 127);
 runIntake(12000);
 fwrd(18,127);
 turn(-33, 127);
 pros::delay(200);
 mogoClamp.set(false);
 runIntake(12000);
 fwrd(40,75);
 runIntake(12000);
 pros::delay(200);
 fwrd(5,127);
 pros::delay(250);
 runIntake(0);
 frontstage.move_voltage(12000);
fwrd(5,127);
runIntake(0);
fwrd(10,127);
pros::delay(100);
turn(50,127);
rvs(30,127);
rvs(3, 55);
mogoClamp.set(true);
pros::delay(150);
turn(-75,127);
runIntake(12000);
fwrd(22,127);
rvs(10,127);
turn(120,127);
lbrd = 315;
fwrd(18,127);




}

void red_right_rush() {
  rvs(34, 127);
  pros::delay(120);
  turn(-40, 100);
  rvs(13, 65);
  mogoClamp.set(true);
  runIntake(12000);
  pros::delay(550);
  turn(10, 127);
  fwrd(15, 127);
  turn(90, 127);
  pros::delay(700);
  mogoClamp.set(false);
  turn(-97, 127);
  rvs(15, 90);
  mogoClamp.set(true);

}

void red_right_win_point() {
  rvs(29, 60);
  mogoClamp.set(true);
  runIntake(12000);
  pros::delay(700);
  turn(-125, 127);
  fwrd(30, 127);
  pros::delay(300);
  turn(-45, 127);
  fwrd(42, 90);
  fwrd(18, 127);
  rvs(12, 90);
  pros::delay(500);
  turn(0, 127);
  
}
void red_right_rush_stateswinpoint() {
  pros::Task ladybrown(lbr);
  lDoinker.set(true);
  fwrd(34, 127);
  lDoinker.set(false);
  pros::delay(200);
  rvs(10, 90);
  lDoinker.set(true);
  pros::delay(100);
  rvs(5, 90);
  lDoinker.set(false);
  turn(160, 127);
  pros::delay(100);
  rvs(18, 50);
  pros::delay(100);
  mogoClamp.set(true);
  pros::delay(250);
  runIntake(12000);
  turn(50, 127);
  fwrd(10, 127);
  rvs(10, 90);
  turn(160, 127);
  mogoClamp.set(false);
  fwrd(5, 127);
  turn(50, 127);
  rvs(10, 50);
  mogoClamp.set(true);
  pros::delay(100);
  turn(140, 127);
  fwrd(30, 127);
  turn(60, 127);
  fwrd(30, 127);
  pros::delay(250);
  rvs(5, 127);
  IntakeLift.set(true);
  fwrd(5, 127);
  pros::delay(250);
  IntakeLift.set(false);
  rvs(45, 127);

}
void red_left_win_point() {
  rvs(18, 100);
  turn(18.5,100);
  rvs(23.8, 65);
  pros::delay(250);
  mogoClamp.set(true);
  pros::delay(350);
  runIntake(12000);
  pros::delay(350);
  turn(125, 80);
  fwrd(25, 65);
  pros::delay(200);
  turn(209, 100);
  fwrd(13.8, 80);
  pros::delay(250);
  rvs(9, 100);
  turn(235, 100);
  fwrd(9.6, 100);
  pros::delay(250);
  rvs(21, 100);
  pros::delay(100);
  turn(48, 100);
  fwrd(60, 80);
  fwrd(10, 100);
  pros::delay(250);
  rvs(15, 70);
  turn(-59, 100);
  pros::delay(2000);
  revRunIntake(12000);
  fwrd(85, 100);
 
}

void red_left_four() {
  
}

void blue_right_win_point() {
  rvs(18, 100);
  turn(-27,100);
  rvs(25.3, 70);
  pros::delay(130);
  mogoClamp.set(true);
  pros::delay(330);
  runIntake(12000);
  pros::delay(260);
  turn(-125, 75);
  fwrd(18.8, 65);
  pros::delay(210);
  turn(-203, 100);
  fwrd(14.7, 80);
  pros::delay(180);
  rvs(9, 100);
  turn(-235, 100);
  fwrd(11.9, 100);
  pros::delay(230);
  rvs(21, 100);
  pros::delay(80);
  turn(-65.5, 100);
  fwrd(42, 90);
  fwrd(8, 110);
  pros::delay(250);
  rvs(15, 75);
  turn(45, 100);
  pros::delay(350);
  revRunIntake(12000);
  fwrd(85, 100);

}

void blue_left_rush() {
  
}

void blue_left_win_point() {
  pros::Task ladybrown(lbr);
  lbrd = 340;
  pros::delay(1000);
  lbrd = 108;
  rvsUntil(18.2, 100, 9.1, 60);
  turn(30,80);
  rvs(25, 70);
  pros::delay(300);
  mogoClamp.set(true);
  pros::delay(300);
  runIntake(12000);
  turn(120, 80);
  fwrd(25, 100);
  pros::delay(200);
  turn(61, 100);
  fwrd(45, 100);
  pros::delay(200);
  rvs(30, 70);
  turn(-65, 70);
  fwrd(60, 30);
  turn(-80, 50);
  lbrd = 340;


}

void blue() {

}

void skills() {
  pros::Task ladybrown(lbr);
  lbrd = 290;
  pros::delay(500);
  rvs(15, 50);
  lbrd = 116;
  pros::delay(200);
  mogoClamp.set(true);
  pros::delay(100);
  turn(130, 80);
  revRunIntake(12000);
  pros::delay(100);
  runIntake(12000);
  fwrd(20, 80);
  turn(151, 100);
  pros::delay(50);
  fwrd(53, 100);
  turn(125, 100);
  rvs(19, 100);
  turn(214.4, 100);
  lbrd = 138;
  fwrd(20, 70);
  pros::delay(600);
  revRunIntake(8000);
  pros::delay(60);
  stopIntake();
  pros::delay(50);
  lbrd = 266;
  pros::delay(400);
  rvs(15.9, 60);
  lbrd = 116; 
  runIntake(12000);
  turn(303.5, 100);
  fwrd(60, 90);
  rvs(21, 100);
  turn(256.5, 100);
  fwrd(15, 100);
  pros::delay(100);
  rvs(12, 100);
  turn(110, 100);
  rvs(23, 100);
  mogoClamp.set(false);
  fwrd(8.5, 80);
  pros::delay(50);
  turn(208, 50);
  rvs(71, 60);
  mogoClamp.set(true);
  pros::delay(100);
  turn(105, 80);
  fwrd(20, 80);
  turn(84, 100);
  pros::delay(50);
  fwrd(56.2, 100);
  turn(95, 100);
  rvs(26.5, 100);
  turn(25, 100);
  lbrd = 138;
  fwrd(21, 70);
  pros::delay(600);
  revRunIntake(8000);
  pros::delay(60);
  stopIntake();
  pros::delay(50);
  lbrd = 266;
  pros::delay(400);
  rvs(16.9, 50);
  lbrd = 116;
  runIntake(12000);
  turn(-60, 100);
  fwrd(60, 100);
  rvs(22, 100);
  turn(15, 100);
  fwrd(17, 100);
  pros::delay(100);
  rvs(12, 100);
  turn(142, 100);
  rvs(17, 100);
  mogoClamp.set(false);
  turn(120, 100);
  fwrd(73, 100);


}

void test() {

}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .