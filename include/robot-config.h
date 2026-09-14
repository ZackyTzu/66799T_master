#pragma once
#include "Template/api.h"

using namespace pros;

class Drive;
extern Drive chassis;

extern IMU inertial;
// extern Rotation fwd_tracker;
// extern Rotation sideways_tracker;

extern Motor leftFront;
extern Motor leftBack;
extern Motor rightFront;
extern Motor rightBack;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern Motor lift1;
extern Motor lift2;

extern Motor toggle;

extern Motor intake;

extern Motor left_roller;
extern Motor right_roller;
extern MotorGroup rollers;

extern Rotation arm_rotation;
extern adi::DigitalOut claw;

extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis_lemlib;

void default_constants();

// ---- Lift positions ----
// In the same units as the rest of the arm code: fabs(arm_rotation.get_position()
// / 100.0), zeroed with the arm fully down in initialize(). The sensor reads
// these levels inverted (356/343/335/315), so they are flipped back here, plus
// 15 deg on every level.
const double lift_level0 = 0;           // fully down, no offset
const double lift_level1 = 360 - 356 + 15;
const double lift_level0_5 = lift_level1 - 2.5;  // level 0.5: 2.5 deg under level 1
const double lift_level2 = 360 - 343 + 15;
const double lift_level3 = 360 - 335 + 22;  // 7 deg above the common offset
const double lift_level4 = 360 - 315 + 15;

void lift_to_degrees(double target_deg, int timeout_ms = 1500);

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::adi::DigitalIn limit_switch('A');