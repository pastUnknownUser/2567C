#include "autons.hpp"
#include <sys/syslimits.h>
#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "liblvgl/font/lv_symbol_def.h" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
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


bool On = false;
bool stuck = false;
bool antiJam = false;
bool manualIntake = false;
bool redNeeded = false;
bool blueNeeded = false;

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

void runIntake() {
  while (true) {
    if (manualIntake == false) {
      if (!On) {
        ///intake.move_voltage(0);
        ///intake.move_voltage(0);
        pros::delay(50);
        continue;
      }
      if ((antiJam = true)) {
        float hookTorque = intake.get_torque();
        if (hookTorque <= 0) {
          stuck = true;
        }
  
        if (stuck == true) {
          //intake.move_voltage(-12000);
          pros::delay(150);
          //intake.move_voltage(0);
          stuck = false;
        }
        if (stuck == false) {
          //intake.move_voltage(12000);
          //intake.move_voltage(12000);
        }
      }
      
      if (On) {
        //intake.move_voltage(12000);
        //intake.move_voltage(12000);
      }
      
    }
    pros::delay(20);
  }
}

void colorSort() {
  colorSensor.set_led_pwm(100);
  while (true) {
    long int ringHue = colorSensor.get_hue(); 
    if ((ringHue < 20) && (redNeeded = true)) {  //red 
      On = false;
      pros::delay(400);
      //intake.set_zero_position(0);
      //intake.move_relative(500, 40);
      On = true;
    }
    if ((ringHue > 180) && (blueNeeded = true)) {  //blue
      On = false;
      pros::delay(400);
      intake.set_zero_position(0);
      intake.move_relative(500, 40);
      On = true;
    }
    pros::delay(10);
  }
}

void revRunIntake(float voltage) {
  intake.move_voltage(-voltage); // intake out
  intake.move_voltage(-voltage);
}

void stopIntake() {
  intake.move_voltage(0);
  intake.move_voltage(0);
}

inline void set_lift(int input) {
  //lbm.move(input);
}

void lbr() {
  Lady.exit_condition_set(100, 3, 400, 7, 100, 200);
  lb.reset();
  while (true) {
    if (PIDenable == 1) {
      Lady.target_set(lbrd);
      lbm1.move(Lady.compute(lb.get_position()/100.0));
      lbm2.move(-Lady.compute(lb.get_position()/100.0));
    }
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
  PIDenable = 1;

  fwrd(4, 110);
  lbrd = 310;
  pros::delay(610);
  rvs(25, 127);
  rvs(12,55);
  lbrd = 116;
  mogoClamp.set(true);
  pros::delay(150);
  turn(123, 127);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  fwrd(18,127);
  turn(-33, 127);
  pros::delay(200);
  mogoClamp.set(false);
  fwrd(40,75);
  pros::delay(190);
  fwrd(5,127);
  pros::delay(240);
  intake.move_voltage(0);
  intake.move_voltage(12000);
  fwrd(15,127);
  pros::delay(90);
  turn(47,127);
  rvs(38,100);
  rvs(6, 65);
  mogoClamp.set(true);
  pros::delay(150);
  turn(-75,127);
  intake.move_voltage(12000);
  fwrd(22,127);
  rvs(10,127);
  turn(120,127);
  lbrd = 315;
  fwrd(18,127);



  ladybrown.remove();
}

void red_right_rush() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  pros::delay(2);
  lbrd = 300;
  pros::delay(650);
  rvs(3,127);
  turn(-40,127);
  lbrd = 120;
  rvs(20,127);
  rvs(11,60);
  mogoClamp.set(true);
  pros::delay(120);
  turn(-157, 100);
  lbrd = 300; 
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  fwrd(25,127);
  pros::delay(200);
  turn(-83,127);
  fwrd(44,65);
  pros::delay(250);
  rvs(25,100);
  lbrd = 210;
  pros::delay(200);
  turn(65,127);
  lbrd = 250;
  intake.move_voltage(-12000);
  fwrd(38,95);
  lbrd = 285;
  

  
  ladybrown.remove();
}

void red_right_Simple_Wmiddlering() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  lbrd = 300;
  pros::delay(400);
  rvs(2,127);
  turn(-43,127);
  lbrd = 120;
  rvs(20,127);
  rvs(18,65);
  mogoClamp.set(true);
  pros::delay(75);
  turn(75,127);
  fwrd(21.5,127);
  intake.move_voltage(-12000);
  rDoinker.set(true);
  pros::delay(50);
   rvs(20,127);
   turn(-40,127);
   rDoinker.set(false);
   pros::delay(50);
   intake.move_voltage(12000);
  intake.move_voltage(12000);
  turn(-18,127);
  fwrd(13,127);
  pros::delay(150);
  IntakeLift.set(true);
  fwrd(13,127);
  IntakeLift.set(false);
  pros::delay(50);
  intake.move_voltage(0);
  pros::delay(50);
  rvs(30,127);
  turn(-135,127);
  intake.move_voltage(12000);
  fwrd(26, 127);
  rvs(7,127);
  turn(-87.5,127);
  lbrd = 300;
  intake.move_voltage(12000);
  fwrdUntil(43, 127, 35, 60);
  pros::delay(100);
  rvs(15,127);
  intake.move_voltage(0);
  turn(65,127);
  fwrd(15,127);




  
  ladybrown.remove();
}

void blue_left_Simple_Wmiddlering() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  lbrd = 300;
  pros::delay(400);
  rvs(4,127);
  turn(43,127);
  lbrd = 120;
  rvs(20,127);
  rvs(18,65);
  mogoClamp.set(true);
  pros::delay(75);
  turn(-77,127);
  fwrd(21.5,127);
  intake.move_voltage(-12000);
  lDoinker.set(true);
  pros::delay(50);
   rvs(23,127);
   turn(40,127);
   lDoinker.set(false);
   pros::delay(50);
   intake.move_voltage(12000);
  intake.move_voltage(12000);
  turn(18,127);
  fwrd(13,127);
  pros::delay(150);
  IntakeLift.set(true);
  fwrd(13,127);
  IntakeLift.set(false);
  pros::delay(50);
  intake.move_voltage(0);
  pros::delay(50);
  rvs(30,127);
  turn(135,127);
  intake.move_voltage(12000);
  fwrd(26, 127);
  rvs(7,127);
  turn(83.5,127);
  lbrd = 300;
  intake.move_voltage(12000);
  fwrdUntil(43, 127, 35, 60);
  pros::delay(100);
  rvs(15,127);
  intake.move_voltage(0);
  turn(-65,127);
  fwrd(15,127);




  
  ladybrown.remove();
}

void blue_left_goal_rush() {
  
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  rDoinker.set(true);
  intake.move_voltage(120000);
  fwrd(37.5, 127);
  rDoinker.set(false);
  pros::delay(150);
  rvs(12.5, 90);
  rDoinker.set(true);
  rvs(5,127);
  rDoinker.set(false);
  pros::delay(100);
  turn(15,127);
  fwrd(6,127);
  pros::delay(100);
  lbrd = 350;
  pros::delay(450);
  fwrd(7,127);
  turn(30, 127);
  rvs(8,127);
  turn(-63,127);
  rvsUntil(16,127,7,50);
  mogoClamp.set(true);
  pros::delay(100);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  turn(-195,127);
  //lbrd = 142;
  IntakeLift.set(true);
  //pros::delay(100);
  //intake.move_voltage(0);
  lbrd = 128;
  fwrd(28.5,127);
  IntakeLift.set(false);
  pros::delay(100);
  rvs(6,100);
  //pros::delay(300);
  intake.move_voltage(10000);
  /*turn(168,127);
  fwrd(18,100);
  intake.move_voltage(0);
  lbrd = 330;
  pros::delay(500);
  revRunIntake(12000);
  rvs(8,127);
  lbrd = 266;*/
  turn(-90,127);
  intake.move_voltage(120000);
  intake.move_voltage(12000);
  lbrd = 300;
  fwrdUntil(50, 127, 38, 45);
  fwrd(8,100);
  pros::delay(100);
  rvs(40,127);
  lbrd= 250;
  turn(65,127);
  fwrd(15,127);
  lbrd = 300;



  
 // IntakeLift.set(true);
  /*
  turn(55,127);
  rvs(10,70);
  mogoClamp.set(true);
  pros::delay(100);
  turn(20,127);
  fwrd(20,127);
    */




  
 // IntakeLift.set(true);
  /*
  turn(55,127);
  rvs(10,70);
  mogoClamp.set(true);
  pros::delay(100);
  turn(20,127);
  fwrd(20,127);
    */

  
  

  ladybrown.remove();
}

void blue_left() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  pros::delay(2000);
  lbrd = 300;
  pros::delay(650);
  rvs(3,127);
  turn(45,127);
  lbrd = 120;
  rvs(20,127);
  rvs(11,60);
  mogoClamp.set(true);
  pros::delay(120);
  turn(155, 100);
  lbrd = 300;
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  fwrd(25,127);
  pros::delay(200);
  turn(83,127);
  fwrd(44,65);
  pros::delay(250);
  rvs(25,80);
  lbrd = 210;
  turn(-65,127);
  lbrd = 250;
  intake.move_voltage(-12000);
  fwrd(38,95);
  lbrd = 290;

  ladybrown.remove();
}

void blue_right_ring_rush() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  //pros::Task intake(runIntake);
  lbrd = 116;
  rDoinker.set(true);
  intake.move_voltage(12000);
  fwrd(40,127);
  pros::delay(100);
  rvs(13, 127);
  rDoinker.set(false);
  turn(80,127);
  rvs(8, 127);
  rvs(10,48);
  pros::delay(100);
  mogoClamp.set(true);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  turn(67,127);
  fwrd(19,127);
  pros::delay(200);
  rvs(10,127);
  turn(84, 127);
  fwrd(29.5, 127);
  pros::delay(100);
  rvs(10,127);
  turn(136,127);
  lbrd = 300;
  fwrdUntil(45, 127, 38, 25);
  pros::delay(100);
  rvs(8,127);
  turn(230,127);
  intake.move_voltage(-12000);
  fwrd(20,127);
  lbrd = 210;
  turn(300, 127);
  lbrd = 266;
  fwrd(35, 127);
  lbrd = 280;

  /*
  intake.move_voltage(0);
  intake.move_voltage(0);
  turn(-289, 127);
  fwrd(10.6, 127);
  lDoinker.set(true);
  pros::delay(120);
  rvs(30, 127);
  turn(62,127);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  lDoinker.set(false);
  lbrd = 266;
  fwrd(15, 100);

*/


















  
  ladybrown.remove();
}




void red_right_win_point() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  rvs(29, 60);
  mogoClamp.set(true);
  On = true;
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
  
  ladybrown.remove();
}
void red_right_rush_stateswinpoint() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  lDoinker.set(true);
  intake.move_voltage(120000);
  fwrd(37.5, 127);
  lDoinker.set(false);
  pros::delay(150);
  rvs(12.5, 90);
  lDoinker.set(true);
  rvs(5,127);
  lDoinker.set(false);
  pros::delay(100);
  turn(-15,127);
  fwrd(6,127);
  pros::delay(100);
  lbrd = 350;
  pros::delay(450);
  fwrd(7,127);
  turn(-30, 127);
  rvs(8,127);
  turn(63,127);
  rvsUntil(16,127,7,50);
  mogoClamp.set(true);
  pros::delay(100);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  turn(195,127);
  //lbrd = 142;
  IntakeLift.set(true);
  //pros::delay(100);
  //intake.move_voltage(0);
  lbrd = 128;
  fwrd(28.5,127);
  IntakeLift.set(false);
  pros::delay(100);
  rvs(6,100);
  //pros::delay(300);
  intake.move_voltage(10000);
  /*turn(168,127);
  fwrd(18,100);
  intake.move_voltage(0);
  lbrd = 330;
  pros::delay(500);
  revRunIntake(12000);
  rvs(8,127);
  lbrd = 266;*/
  turn(90,127);
  intake.move_voltage(120000);
  intake.move_voltage(12000);
  lbrd = 300;
  fwrdUntil(50, 127, 38, 45);
  fwrd(8,100);
  pros::delay(100);
  rvs(40,127);
  lbrd = 210;
  turn(-65,127);
  fwrd(15,127);
  lbrd = 300;



  
 // IntakeLift.set(true);
  /*
  turn(55,127);
  rvs(10,70);
  mogoClamp.set(true);
  pros::delay(100);
  turn(20,127);
  fwrd(20,127);
    */

  
  ladybrown.remove();
}

void red_left_four() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  //pros::Task intake(runIntake);
  lbrd = 116;
  lDoinker.set(true);
  intake.move_voltage(12000);
  fwrd(40,127);
  pros::delay(100);
  rvs(13, 127);
  lDoinker.set(false);
  turn(-80,127);
  rvs(8, 127);
  rvs(10,48);
  pros::delay(100);
  mogoClamp.set(true);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  turn(-67,127);
  fwrd(19,127);
  pros::delay(200);
  rvs(10,127);
  turn(-84, 127);
  fwrd(29.5, 127);
  pros::delay(100);
  rvs(10,127);
  turn(-136,127);
  lbrd = 300;
  fwrdUntil(45, 127, 38, 25);
  pros::delay(100);
  rvs(8,127);
  turn(-230,127);
  fwrd(20,127);
  lbrd = 210;
  turn(-300, 127);
  lbrd = 266;
  fwrd(35, 127);
  lbrd = 280;

  /*
  intake.move_voltage(0);
  intake.move_voltage(0);
  turn(-289, 127);
  fwrd(10.6, 127);
  lDoinker.set(true);
  pros::delay(120);
  rvs(30, 127);
  turn(62,127);
  intake.move_voltage(12000);
  intake.move_voltage(12000);
  lDoinker.set(false);
  lbrd = 266;
  fwrd(15, 100);

*/










  ladybrown.remove();
}

void blue_right_win_point() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  rvs(18, 100);
  turn(-27,100);
  rvs(25.3, 70);
  pros::delay(130);
  mogoClamp.set(true);
  pros::delay(330);
  On = true;
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

  ladybrown.remove();
}

void blue_left_rush() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  

  ladybrown.remove();
}

void blue_left_win_point() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  lbrd = 340;
  pros::delay(1000);
  lbrd = 108;
  rvsUntil(18.2, 100, 9.1, 60);
  turn(30,80);
  rvs(25, 70);
  pros::delay(300);
  mogoClamp.set(true);
  pros::delay(300);
  On = true;
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

  ladybrown.remove();
}

void blue() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;

}

void skills() {
  pros::Task ladybrown(lbr);
  PIDenable = 1;
  pros::Task intake(runIntake);
  lbrd = 290;
  pros::delay(500);
  rvs(15, 50);
  lbrd = 116;
  pros::delay(200);
  mogoClamp.set(true);
  pros::delay(100);
  turn(130, 80);
  manualIntake = true;
  revRunIntake(12000);
  pros::delay(100);
  manualIntake = false;
  On = true;
  fwrd(18, 80);
  turn(151, 100);
  pros::delay(50);
  fwrd(56, 90);
  turn(138, 100);
  rvs(20.4, 100);
  turn(214.4, 100);
  lbrd = 138;
  fwrd(20, 70);
  pros::delay(750);
  On = false;
  pros::delay(90);
  lbrd = 266;
  pros::delay(400);
  rvs(15.8, 60);
  lbrd = 116; 
  On = true;
  turn(304.9, 100);
  fwrd(60, 80);
  rvs(19, 100);
  turn(247, 100);
  fwrd(15, 100);
  pros::delay(100);
  rvs(12, 100);
  turn(92, 100);
  rvs(23, 100);
  mogoClamp.set(false);
  fwrd(8.4, 80);
  pros::delay(50);
  turn(213, 50);
  rvs(73, 60);
  mogoClamp.set(true);
  pros::delay(100);
  fwrd(3, 100);
  turn(105.5, 80);
  fwrd(27, 80);
  rvs(11.6, 100);
  turn(90, 100);
  pros::delay(50);
  fwrd(65, 100);
  turn(60, 100);
  rvs(10, 100);
  turn(129, 100);
  rvs(17.3, 100);
  turn(27, 100);
  lbrd = 138;
  fwrd(26, 70);
  pros::delay(750);
  On = false;
  pros::delay(90);
  lbrd = 266;
  pros::delay(400);
  rvs(15.6, 50);
  lbrd = 116;
  manualIntake = false;
  On = true;
  turn(-60, 100);
  fwrd(65, 85);
  rvs(22, 127);
  turn(14, 127);
  fwrd(17, 127);
  pros::delay(100);
  rvs(12, 127);
  turn(149, 127);
  rvs(19, 127);
  mogoClamp.set(false);
  fwrd(8, 127);
  turn(120, 127);
  fwrd(73, 127);
  turn(167, 127);
  fwrd(64, 127);
  turn(40, 127);
  fwrd(70, 127);
  rvs(35, 127);
  turn(180, 127);
  fwrd(70, 127);
  turn(160, 127);
  fwrd(75, 127);
  rvs(20, 127);
  lbrd = 266;
  turn(160, 127);
  rvs(60, 90);
  fwrd(15, 127);
  lbrd = 145;

}


void test() {
  colorSensor.set_led_pwm(100);
  pros::Task intake(runIntake);
  pros::Task sorter(colorSort);
  pros::delay(1000);
  blueNeeded = true;
  antiJam = false;
  On = true;

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
