#include "main.h"

IMU inertial(3);
// No tracking wheels (DriveStyle::ZERO_TRACKER), so the tracker sensors are
// commented out, along with their Drive constructor arguments below.
// Rotation fwd_tracker(9);
// Rotation sideways_tracker(10);

// 66799T
// negative port number means reversed (there is no separate "reversed" constructor argument)
Motor leftFront(-7, MotorGears::blue);
Motor leftBack(4, MotorGears::blue);
Motor rightFront(2, MotorGears::blue);
Motor rightBack(-1, MotorGears::blue);

MotorGroup leftMotors({leftFront.get_port(),leftBack.get_port()});
MotorGroup rightMotors({rightFront.get_port(), rightBack.get_port()});

Motor lift1(-18,MotorGears::red);
Motor lift2(12,MotorGears::red);

Motor toggle(-16,MotorGears::green);

Motor left_roller(-13,MotorGears::green);
Motor right_roller(19,MotorGears::green);

MotorGroup rollers({left_roller.get_port(), right_roller.get_port()});

Rotation arm_rotation(-14);

adi::DigitalOut claw('H');



Drive chassis(
    // Drive Style (see drive.h for option list): 
    Drive::DriveStyle::ZERO_TRACKER,
    
    // Left Motors:
    leftMotors,

    // Right Motors:
    rightMotors,

    // Your inertial sensor:
    inertial,

    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    2.75,

    // External ratio, input teeth/output teeth. This used to be written as
    // 48/36, which is INTEGER division and evaluates to exactly 1 (36/48 gave
    // 0, which is why odom "barely moved" with it). Every drive distance and
    // PID constant below was tuned against 1, so 1 is kept explicitly here.
    1,

    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    360,

    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    // fwd_tracker,

    // Input the Forward Tracker diameter (reverse it to make the direction switch)
    // For a tank drive using odom without a forward tracker, this value is useless and does not affect anything:
    // 2,

    // Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
    // This distance is in inches:
    5.25,

    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    // sideways_tracker,

    // Sideways tracker diameter (reverse it to make the direction switch):
    // 0,

    // Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
    0
);


void default_constants(){
    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI(, minVoltage)).
    chassis.set_drive_constants(127, 12, 0.1, 1, 5, 0);
    chassis.set_heading_constants(64, 1.65, 0, 8, 0);
    chassis.set_turn_constants(107, 2.5, .10583, 17.4625, 15.0);
    chassis.set_swing_constants(127, 3.704166667, 0.08466667, 21.1666667, 15);
    
    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_drive_exit_conditions(1.875, 45, 3000);
    chassis.set_turn_exit_conditions(1.9, 45, 2000);
    chassis.set_swing_exit_conditions(1.82, 45, 2000);
    
    // (min_voltage, early_exit_distance)
    chassis.set_drive_motion_chain_constants(60, 4);
    chassis.set_turn_motion_chain_constants(50, 5);
}

/**
 * Drives the lift to target_deg and holds it there. Blocking: it returns once
 * the arm has settled within 1 deg or timeout_ms has passed, whichever is
 * first. Call it with lift_level1..lift_level4 from an auton routine.
 */
void lift_to_degrees(double target_deg, int timeout_ms){
  lift1.set_brake_mode(MotorBrake::hold);
  lift2.set_brake_mode(MotorBrake::hold);

  // Same feedforward opcontrol uses to stop the arm sagging, so the PID does
  // not have to carry gravity with a standing error.
  const double lift_hold_ff = 12;
  // Raising fights gravity, lowering is helped by it, so the down direction is
  // capped lower to keep the arm from slamming.
  const double lift_up_voltage = 127;
  const double lift_down_voltage = 60;
  double error = target_deg - fabs(arm_rotation.get_position() / 100.0);
  // (error, kp, ki, kd, starti, settle_error, settle_time, timeout)
  // kp/kd are starting points -- tune them on the robot.
  PID liftPID(error, 4, 0, 10, 0, 1, 100, timeout_ms);

  while(!liftPID.is_settled()){
    error = target_deg - fabs(arm_rotation.get_position() / 100.0);
    double output = clamp(liftPID.compute(error) + lift_hold_ff, -lift_down_voltage, lift_up_voltage);
    lift1.move(output);
    lift2.move(output);
    delay(10);
  }

  lift1.brake();
  lift2.brake();
}

// ---- LemLib (only chassis_lemlib.setPose() is used, from Drive::set_coordinates) ----

lemlib::Drivetrain drivetrain(
    &leftMotors, // left motors
    &rightMotors, // right motors
    10.5, // track width in inches
    2.75, // wheel diameter in inches
    800, // rpm of the wheels
    2 // horizontal drift 
);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    &inertial // inertial sensor
);

// lateral PID controller. Only slew is used in Pure Pursuit. 
lemlib::ControllerSettings lateral_controller(
    chassis.drive_kp, // proportional gain (kP)
    chassis.drive_ki, // integral gain (kI)
    chassis.drive_kd, // derivative gain (kD)
    chassis.drive_starti, // anti windup (= startI)
    1, // small error range, in inches
    100000000000, // small error range timeout, in milliseconds // lemlib default is 100
    chassis.drive_settle_error, // large error range, in inches
    chassis.drive_settle_time, // large error range timeout, in milliseconds 
    0 // maximum acceleration (slew) // TODO: tune this if necessary
);

// angular PID controller. Not used in Pure Pursuit. 
lemlib::ControllerSettings angular_controller(
    chassis.turn_kp, // proportional gain (kP)
    chassis.turn_ki, // integral gain (kI)
    chassis.turn_kd, // derivative gain (kD)
    chassis.turn_starti, // anti windup (= startI)
    1, // small error range, in degrees
    100000000000, // small error range timeout, in milliseconds // lemlib default is 100
    chassis.turn_settle_error, // large error range, in degrees
    chassis.turn_settle_time, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis_lemlib(
    drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors // odometry sensors
);
