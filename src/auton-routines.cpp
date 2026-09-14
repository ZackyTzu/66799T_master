#include "main.h"

// ---- Routines (selected on the dashboard's Auton Select tab) ----
int lift_l1(){
  lift_to_degrees(lift_level1);
  return 0;
}

int lift_l0(){
  lift_to_degrees(lift_level0);
  return 0;
}

int lift_l2(){
  lift_to_degrees(lift_level2);
  delay(250);
  rollers.move(20);
  return 0;
}

int lift_l2_down(){
  lift_to_degrees(lift_level2);;
  return 0;
}

int lift_l3(){
  lift_to_degrees(lift_level3);
  delay(250);
  rollers.move(20);
  return 0;
}

int lift_l0_5(){
  lift_to_degrees(lift_level0_5);
  return 0;
}


void left(){
  default_constants();
  toggle.move(127);
  delay(930);
  toggle.brake();
  Task lift_1(lift_l1);
  chassis.drive_distance(12);
  chassis.turn_to_angle(85);
  chassis.drive_with_voltage(42.5,42.5);
  delay(750);
  chassis.drive_stop(MotorBrake::brake);
  delay(100);
  Task lift_0(lift_l0);
  delay(200);
  claw.set_value(true);
  delay(100);
  chassis.drive_max_voltage = 100;
  chassis.drive_distance(-7.5);
  chassis.turn_to_angle(37.5);

  default_constants();
  Task lift_00(lift_l0);
  rollers.move(127);
  chassis.drive_max_voltage = 120;
  chassis.drive_distance(21.85);
  chassis.drive_max_voltage = 35;
  chassis.drive_distance(5.65);
  claw.set_value(false);
  chassis.drive_stop(MotorBrake::brake);
  delay(300);
  Task lift_1_2(lift_l1);
  delay(50);
  chassis.turn_to_angle(166);
  Task lift_2(lift_l2);
  chassis.drive_with_voltage(50,50);
  delay(750);
  chassis.drive_stop(MotorBrake::brake);
   delay(100);
  Task lift_0_5(lift_l0_5);
  delay(25);
  delay(150);
  claw.set_value(true);
  delay(200);
  chassis.drive_stop(MotorBrake::brake);
  chassis.drive_distance(-7.5,170,false);
  Task lift_0_3(lift_l0);

  default_constants();
  chassis.turn_to_angle(124);
  rollers.move(127);
  chassis.drive_distance(22.25);
  chassis.drive_max_voltage = 35;
  chassis.drive_distance(6);
  chassis.drive_max_voltage = 127;
  claw.set_value(false);
  chassis.drive_stop(MotorBrake::brake);
  delay(300);
  Task lift_1_3(lift_l1);
  delay(50);
  Task lift_3(lift_l3);
  chassis.turn_to_angle(257);
  chassis.drive_with_voltage(50,50);
  delay(800);
  chassis.drive_stop(MotorBrake::brake);
  delay(100);
  Task lift_2_2(lift_l2_down);
  delay(100);
  claw.set_value(true);
  delay(200);
  Task lift_0_4(lift_l0);
  // chassis.drive_distance(-8,272.5,false);
    chassis.drive_distance(-10);
  default_constants();
  // chassis.drive_with_voltage(-60,-60);
  // delay(2000);
  // chassis.drive_max_voltage = 127;
  // chassis.drive_distance(5);
  // chassis.turn_to_angle(177.5);
  // chassis.drive_distance(7);
  chassis.drive_stop(MotorBrake::brake);

  // chassis.turn_to_angle(333);
  // chassis.drive_max_voltage = 127;
  // chassis.drive_distance(30);
  // rollers.move(127);
  // chassis.drive_max_voltage = 35;
  // chassis.drive_distance(10);
  // claw.set_value(false);
  // rollers.move(20);
  // delay(200);
  // // Task lift_1_5(lift_l1);
  // chassis.drive_max_voltage = 90;
  // chassis.drive_distance(15,0,false);
  // chassis.drive_stop(MotorBrake::brake);
  // lift1.set_brake_mode(MotorBrake::hold);
  // lift2.set_brake_mode(MotorBrake::hold);
  // lift1.brake();
  // lift2.brake();
}

void left2(){
  default_constants();
  chassis.drive_stop(MotorBrake::brake);
}

void right(){
  default_constants();
  chassis.drive_stop(MotorBrake::brake);
}

void right2(){
  default_constants();
  chassis.drive_stop(MotorBrake::brake);
}

void skill(){
  default_constants();
  chassis.drive_stop(MotorBrake::brake);
}

