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

extern Motor left_roller;
extern Motor right_roller;

extern Rotation arm_rotation;
extern adi::DigitalOut claw;

extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis_lemlib;

void default_constants();

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::adi::DigitalIn limit_switch('A');