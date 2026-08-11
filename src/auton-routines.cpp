#include "main.h"

const int CASCADE_SCORE_VELOCITY = 110;   // move_absolute() speed for score(), out of 200 rpm
const int CASCADE_SCORE_TIMEOUT_MS = 3000; // give up waiting and move on after this long
// "Arrived" comes from the one global CASCADE_SETTLE_ERROR (cascade.cpp), the
// same tolerance the cascade PID and the teleop presets settle on -- this used
// to be a separate const CASCADE_SCORE_SETTLE_DEG = 20 that had to be kept in
// sync by hand.
void score(ScoringLevel level, ArmPosition arm_pos, int cascade_velocity){
  int cascade_target = (int)level;

  arm_set_position(arm_pos);
  cascade1.move_absolute(cascade_target, cascade_velocity);
  cascade2.move_absolute(cascade_target, cascade_velocity);

  int waited_ms = 0;
  while(!arm_settled || fabs(cascade1.get_position() - cascade_target) > CASCADE_SETTLE_ERROR){
    delay(10);
    waited_ms += 10;
    if(waited_ms > CASCADE_SCORE_TIMEOUT_MS) break;
  }
}

int cascade_level_2_lift(){
score(ScoringLevel::LEVEL_2, ArmPosition::DOWN);
return 0;
}

int wait_claw_open(){
  delay(400);
  claw.set_value(false);
  return 0;
}

int wait_claw_open2(){
  delay(200);
  claw.set_value(false);
  score(ScoringLevel::LEVEL_2, ArmPosition::DOWN);
  return 0;
}

int cascade_level_3_lift(){
score(ScoringLevel::LEVEL_3, ArmPosition::DOWN);
return 0;
}

int cascade_level_3_lift_arm(){
score(ScoringLevel::LEVEL_3, ArmPosition::POS_4);
return 0;
}

// Runs the arm at the full 127 instead of ARM_DOWN_MAX_VOLTAGE (77), so it
// drops to DOWN as fast as the motor allows. The override is cleared once
// score() returns (it blocks until the arm settles or times out), putting the
// normal caps back for everything else.
int cascade_level_0(){
arm_max_voltage_override = 127;
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
arm_max_voltage_override = 0;
return 0;
}

int cascade_level_opt(){
score(ScoringLevel::LEVEL_opt, ArmPosition::DOWN);
return 0;
}

int cascade_level_tpf(){
score(ScoringLevel::LEVEL_tpf, ArmPosition::DOWN_HOLD, 200);
return 0;
}

int cascade_level_tpf_2(){
score(ScoringLevel::LEVEL_tpf, ArmPosition::DOWN, 200);
return 0;
}

int cascade_level_4(){
score(ScoringLevel::LEVEL_4, ArmPosition::DOWN_HOLD, 200);
return 0;
}

// How long to keep waiting for the arm to physically reach DOWN after score()
// returns, before giving up and restoring the normal caps anyway.
const int LEVEL_1_ARM_DOWN_TIMEOUT_MS = 3000;

// Same override trick as cascade_level_0(), at 117 instead of 127 -- the arm
// drops to DOWN faster than ARM_DOWN_MAX_VOLTAGE (77) would allow.
//
// score()'s wait now covers the arm properly: arm_set_position() clears
// arm_settled synchronously, so the flag is no longer stale-true for the
// previous target and score() can't sail past its arm half. (It used to, and
// since LEVEL_1 is only a 200 degree cascade move, score() would return in
// under a second -- long before the arm had fallen from POS_1 -- dropping the
// rest of the descent back to 77.) The wait below is now just a backstop on the
// measured angle before the caps go back.
int cascade_level_1(){
arm_max_voltage_override = 117;
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);

int waited_ms = 0;
while(fabs(arm_get_position_deg() - arm_target_degrees(ArmPosition::DOWN)) > ARM_SETTLE_ERROR){
  delay(10);
  waited_ms += 10;
  if(waited_ms > LEVEL_1_ARM_DOWN_TIMEOUT_MS) break;
}

arm_max_voltage_override = 0;
return 0;
}

// Rotates ONLY the arm down to 0 (ArmPosition::DOWN) at 125, leaving the
// cascade wherever it is -- unlike cascade_level_0(), which drives both.
// Waits on the measured angle (equivalent to arm_settled with
// ARM_SETTLE_TIME_MS at 0, and independent of it if that gets raised), then
// restores the normal caps. Start it as a Task to overlap it with driving;
// call it directly to block.
int arm_down_fast(){
arm_max_voltage_override = 125;
arm_set_position(ArmPosition::DOWN);

int waited_ms = 0;
while(fabs(arm_get_position_deg() - arm_target_degrees(ArmPosition::DOWN)) > ARM_SETTLE_ERROR){
  delay(10);
  waited_ms += 10;
  if(waited_ms > LEVEL_1_ARM_DOWN_TIMEOUT_MS) break;
}

arm_max_voltage_override = 0;
return 0;
}

int arm_back(){
score(ScoringLevel::LEVEL_1, ArmPosition::ARM_BACK_2);
return 0;
}

int arm_back2(){
score(ScoringLevel::LEVEL_0, ArmPosition::ARM_BACK_2);
return 0;
}

void left(){
// default_constants();
// toggle.set_value(true);
// delay(230);
// chassis.drive_with_voltage(-57,-57);
// delay(600);
// Task arm_down_task = Task(arm_down_fast);
// Task level_1_task = Task(cascade_level_1);
// chassis.drive_distance(16,false);
// chassis.turn_to_angle(71.75);
// chassis.drive_distance(9.75);
default_constants();
toggle.set_value(true);
delay(225);
chassis.drive_with_voltage(-50,-50);
delay(525);
Task arm_down_task = Task(arm_down_fast);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(16,false);
chassis.turn_to_angle(71.5);
chassis.drive_distance(9.75);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-5);
Task level_0_task = Task(cascade_level_0);
chassis.turn_to_angle(124.5);
chassis.drive_max_voltage = 107;
chassis.drive_distance(12.85);
chassis.swing_max_voltage = 90;
chassis.drive_timeout = 1000;
chassis.swing_settle_error = 1.9;
chassis.swing_to_angle(150,true,false);
chassis.drive_timeout = 300;
chassis.drive_settle_error = 2;
chassis.drive_distance(2);
claw.set_value(true);
delay(50);


// chassis.turn_to_angle(140.75,true);
// chassis.drive_max_voltage = 107;
// chassis.drive_timeout = 600;
// chassis.drive_settle_error = 2;
// chassis.drive_distance(5.25);
// chassis.turn_to_angle(149.5);
// chassis.drive_timeout = 300;
// chassis.drive_distance(2.5);
// claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 127;
chassis.drive_distance(-1);
Task level_2_task = Task(cascade_level_2_lift);
chassis.turn_to_angle(105);
chassis.drive_distance(-14.5);
chassis.turn_to_angle(53);
chassis.drive_distance(10);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(300);
chassis.drive_distance(-5);
chassis.turn_to_angle(90);
chassis.drive_distance(-30,81,false);
// chassis.drive_settle_error = 6;
// chassis.drive_settle_time = 20;
// chassis.drive_timeout = 550;
// chassis.wall_distance(Drive::WallSide::RIGHT, -30, 79, 405, 40);
Task cascade0_task = Task(cascade_level_0);

default_constants();
chassis.drive_with_voltage(-40,-40);
delay(500);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_to_angle(80);
chassis.drive_distance(7);
chassis.turn_to_angle(205);

default_constants();
chassis.drive_distance(15.75);
chassis.swing_max_voltage = 90;
chassis.swing_timeout = 1000;
chassis.swing_settle_error = 2;
chassis.swing_to_angle(177.5,false,false);
chassis.drive_timeout = 500;
chassis.drive_settle_error = 2.5;
chassis.drive_distance(3.5);
claw.set_value(true);
delay(50);
// chassis.turn_to_angle(194,true);
// chassis.drive_timeout = 600;
// chassis.drive_settle_error = 2;
// chassis.drive_distance(5.5);
// chassis.turn_to_angle(181);
// chassis.drive_timeout = 300;
// chassis.drive_distance(2.5);
// claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 127;
chassis.drive_distance(-1);
Task level_3_task = Task(cascade_level_3_lift);
chassis.turn_to_angle(205);
chassis.drive_distance(-14.75);
Task level_3_task2 = Task(cascade_level_3_lift_arm);
chassis.turn_to_angle(264.75);
chassis.drive_timeout = 750;
chassis.drive_distance(8.75);
Task claw_task2 = Task(wait_claw_open2);
Task level_3_task3 = Task(cascade_level_3_lift);
delay(400);
chassis.drive_distance(-5);
chassis.drive_stop(MotorBrake::brake);
}

void left2(){
default_constants();
toggle.set_value(true);
delay(225);
chassis.drive_with_voltage(-50,-50);
delay(525);
Task arm_down_task = Task(arm_down_fast);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(16,false);
chassis.turn_to_angle(71.5);
chassis.drive_distance(9.75);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6.75);
chassis.turn_to_angle(14.15);
chassis.drive_max_voltage = 127;
chassis.drive_distance(19);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.85);
delay(50);
claw.set_value(true);
delay(50);

default_constants();
Task level_2_task = Task(cascade_level_2_lift);
default_constants();
chassis.drive_distance(-1);
chassis.turn_max_voltage = 70;
chassis.turn_to_angle(154);
chassis.drive_distance(10.85);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(350);

default_constants();
chassis.drive_distance(-10.85);
Task cascade0_task2 = Task(cascade_level_0);
chassis.turn_to_angle(107.5);
chassis.drive_distance(24);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.25);
delay(100);
claw.set_value(true);
delay(150);
arm_set_position(ArmPosition::DOWN_HOLD);
Task level_tpf_task = Task(cascade_level_tpf);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(246.5);

default_constants();
chassis.drive_max_voltage = 97;
chassis.drive_distance(12);
delay(250);
arm_set_position(ArmPosition::DOWN);
Task claw_task2 = Task(wait_claw_open);
Task level_2_task2 = Task(cascade_level_2_lift);
// delay(1000);
// chassis.drive_distance(-5);
// chassis.drive_stop(MotorBrake::brake);
delay(550);

default_constants();
Task cascade0_task3 = Task(cascade_level_0);
chassis.drive_distance(-7.75);
arm_set_position(ArmPosition::DOWN);
chassis.turn_to_angle(200.5);
chassis.drive_distance(17);
chassis.swing_max_voltage = 90;
chassis.swing_timeout = 1250;
chassis.swing_to_angle(176,false,false);
chassis.drive_timeout = 350;
chassis.drive_distance(2);
// chassis.turn_to_angle(19,true);
// chassis.drive_max_voltage = 77;
// chassis.drive_timeout = 450;
// chassis.drive_distance(4.75);
// chassis.turn_to_angle(178);
delay(150);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_4_task = Task(cascade_level_4);
chassis.turn_to_angle(208);
chassis.drive_distance(-19.5);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(252.5);
chassis.drive_max_voltage = 117;
chassis.drive_distance(6);
delay(100);
arm_set_position(ArmPosition::DOWN);
delay(100);
claw.set_value(false);
chassis.drive_max_voltage = 37;
chassis.drive_distance(-3);
chassis.drive_stop(MotorBrake::brake);
}

void right(){
default_constants();
toggle.set_value(true);
delay(150);
chassis.drive_with_voltage(-44,-44);
delay(450);
Task arm_down_task = Task(arm_down_fast);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(15.5,false);
chassis.turn_to_angle(288);
chassis.drive_distance(11.65);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6);
chassis.turn_to_angle(236);
chassis.drive_max_voltage = 107;
chassis.drive_distance(16);
chassis.turn_to_angle(221.5,true);
chassis.drive_max_voltage = 107;
chassis.drive_timeout = 700;
chassis.drive_settle_error = 2;
chassis.drive_distance(5.45);
chassis.turn_to_angle(210);
chassis.drive_timeout = 400;
chassis.drive_distance(3.5);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_2_task = Task(cascade_level_2_lift);
chassis.turn_to_angle(255);
chassis.drive_distance(-12);
chassis.turn_to_angle(306.75);
chassis.drive_distance(8.65);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(300);
chassis.drive_distance(-5);
chassis.turn_to_angle(276.75);
Task cascade0_task = Task(cascade_level_0);
chassis.drive_distance(-30,282);
// chassis.drive_settle_error = 6.5;
// chassis.drive_settle_time = 20;
// chassis.drive_timeout = 550;
// chassis.wall_distance(Drive::WallSide::LEFT, -30, 281, 405, 50);

default_constants();
chassis.drive_with_voltage(-40,-40);
delay(550);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_to_angle(281);
chassis.drive_distance(7.25);
chassis.turn_to_angle(151);

default_constants();
chassis.drive_distance(17);
chassis.turn_to_angle(165.5,true);
chassis.drive_timeout = 700;
chassis.drive_settle_error = 2;
chassis.drive_distance(5.25);
chassis.turn_to_angle(177);
chassis.drive_timeout = 350;
chassis.drive_distance(3);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 127;
chassis.drive_distance(-1);
Task level_3_task = Task(cascade_level_3_lift);
chassis.turn_to_angle(155);
chassis.drive_distance(-15);
Task level_3_task2 = Task(cascade_level_3_lift_arm);
chassis.turn_to_angle(97);
chassis.drive_timeout = 650;
chassis.drive_distance(8);
Task claw_task2 = Task(wait_claw_open2);
Task level_3_task3 = Task(cascade_level_3_lift);
chassis.drive_distance(-2);
chassis.drive_stop(MotorBrake::brake);
}

void right2(){
default_constants();
toggle.set_value(true);
delay(150);
chassis.drive_with_voltage(-50,-50);
delay(450);
Task arm_down_task = Task(arm_down_fast);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(15.5,false);
chassis.turn_to_angle(287);
chassis.drive_distance(9.85);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6.75);
chassis.turn_to_angle(343.5);
chassis.drive_max_voltage = 127;
chassis.drive_distance(19);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.9);
delay(100);
claw.set_value(true);

default_constants();
Task level_2_task = Task(cascade_level_2_lift);
default_constants();
chassis.drive_distance(-1);
chassis.turn_max_voltage = 70;
chassis.turn_to_angle(207);
chassis.drive_distance(11.5);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(500);

default_constants();
chassis.drive_distance(-11.25);
Task cascade0_task = Task(cascade_level_0);
chassis.turn_to_angle(251.85);
chassis.drive_distance(24.25);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.90);
delay(125);
claw.set_value(true);
delay(200);
arm_set_position(ArmPosition::DOWN_HOLD);
Task level_tpf_task = Task(cascade_level_tpf);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(116.75);

default_constants();
chassis.drive_max_voltage = 107;
chassis.drive_distance(11.75);
delay(250);
arm_set_position(ArmPosition::DOWN);
Task claw_task2 = Task(wait_claw_open);
Task level_2_task2 = Task(cascade_level_2_lift);
delay(500);
chassis.drive_distance(-5);
chassis.turn_to_angle(30);
chassis.drive_distance(-20);
chassis.drive_with_voltage(-47,-47);
delay(200);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_to_angle(118.5);
intake.move(-127);
chassis.turn_max_voltage = 107;
chassis.drive_distance(102,110,false);
delay(50);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_max_voltage = 127;
Task cascade0_task4 = Task(cascade_level_0);
chassis.drive_distance(-100);
chassis.drive_stop(MotorBrake::brake);
// delay(600);

// default_constants();
// Task cascade0_task3 = Task(cascade_level_0);
// chassis.drive_distance(-6.5);
// arm_set_position(ArmPosition::DOWN);
// chassis.turn_to_angle(159.5);
// chassis.drive_distance(20);
// chassis.turn_to_angle(168.5,true);
// chassis.drive_max_voltage = 77;
// chassis.drive_timeout = 450;
// chassis.drive_distance(4.75);
// chassis.turn_to_angle(182);
// delay(150);
// claw.set_value(true);

// default_constants();
// chassis.drive_max_voltage = 127;
// chassis.drive_distance(-1);
// Task level_4_task = Task(cascade_level_4);
// chassis.turn_to_angle(152);
// chassis.drive_distance(-20);
// chassis.turn_max_voltage = 70;
// chassis.turn_to_angle(107.75);
// chassis.drive_max_voltage = 127;
// chassis.drive_distance(6.85);
// arm_set_position(ArmPosition::DOWN);
// delay(100);
// claw.set_value(false);
// delay(200);
// chassis.drive_distance(-3);
// chassis.drive_stop(MotorBrake::brake);
}

void skill(){
default_constants();
toggle.set_value(true);
delay(225);
chassis.drive_with_voltage(-47,-47);
delay(550);
Task arm_down_task = Task(arm_down_fast);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(16,false);
chassis.turn_to_angle(71.65);
chassis.drive_distance(9.75);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
// toggle.set_value(true);
// // Task cascade0_task = Task(cascade_level_0);
// chassis.drive_with_voltage(-57,-57);
// delay(450);
// Task level_1_task = Task(cascade_level_1);
// chassis.drive_distance(16.5,false);
// chassis.turn_to_angle(72);
// chassis.drive_distance(11.5);
// score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
// delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6.75);
chassis.turn_to_angle(14.15);
chassis.drive_max_voltage = 127;
chassis.drive_distance(19);
chassis.drive_max_voltage = 20;
chassis.drive_distance(7);
delay(50);
claw.set_value(true);
delay(50);

default_constants();
Task level_2_task = Task(cascade_level_2_lift);
default_constants();
chassis.drive_distance(-1);
chassis.turn_max_voltage = 70;
chassis.turn_to_angle(154);
chassis.drive_distance(10.85);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(350);

default_constants();
chassis.drive_distance(-10.85);
Task cascade0_task2 = Task(cascade_level_0);
chassis.turn_to_angle(107.5);
chassis.drive_distance(24);
chassis.drive_max_voltage = 20;
chassis.drive_distance(7);
delay(250);
claw.set_value(true);
delay(250);
arm_set_position(ArmPosition::DOWN_HOLD);
Task level_tpf_task = Task(cascade_level_tpf);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(244);

default_constants();
chassis.drive_max_voltage = 97;
chassis.drive_distance(12);
delay(250);
arm_set_position(ArmPosition::DOWN);
Task claw_task2 = Task(wait_claw_open);
Task level_2_task2 = Task(cascade_level_2_lift);
delay(1000);
Task cascade0_task3 = Task(cascade_level_0);
chassis.drive_distance(-15,255,false);
chassis.drive_with_voltage(-57,-57);
delay(1000);
chassis.drive_distance(5.15);
chassis.turn_to_angle(164.5);
chassis.drive_distance(13,163,true);

default_constants();
chassis.drive_max_voltage = 25;
chassis.drive_distance(12.5);
delay(100);
claw.set_value(true);
chassis.drive_max_voltage = 97;
Task level_1_task2 = Task(cascade_level_1);
chassis.drive_distance(-10);
Task level_4_task = Task(cascade_level_4);
chassis.drive_distance(-10);
chassis.turn_to_angle(250);
chassis.drive_distance(25.75);
arm_set_position(ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(300);
Task cascade0_task4 = Task(cascade_level_0);
chassis.drive_distance(-24.85);
chassis.turn_to_angle(164);
delay(1000);
chassis.drive_distance(12.5,163,true);

default_constants();
chassis.drive_max_voltage = 25;
chassis.drive_distance(12.5);
delay(100);
claw.set_value(true);
chassis.drive_max_voltage = 97;
Task level_1_task3 = Task(cascade_level_1);
chassis.drive_distance(-13);
chassis.turn_to_angle(114);
Task arm_back_task = Task(arm_back2);
chassis.drive_distance(-60);
chassis.drive_with_voltage(-27,-27);
delay(2500);
chassis.drive_stop(MotorBrake::brake);
delay(100);
claw.set_value(false);
delay(500);
chassis.turn_to_angle(113.5);
Task cascade0_task5 = Task(cascade_level_0);
chassis.drive_distance(65);
chassis.turn_to_angle(255);
chassis.drive_with_voltage(-57,-57);
delay(1000);
chassis.drive_distance(5.15);
chassis.turn_to_angle(164.5);
chassis.drive_distance(13,163,true);

default_constants();
chassis.drive_max_voltage = 25;
chassis.drive_distance(12.65);
delay(100);
claw.set_value(true);
chassis.drive_max_voltage = 97;
Task level_1_task4 = Task(cascade_level_1);
chassis.drive_distance(-13.25);
chassis.turn_to_angle(114);
Task arm_back_task3 = Task(arm_back);
chassis.drive_distance(-60);
chassis.drive_with_voltage(-27,-27);
delay(2300);
chassis.drive_stop(MotorBrake::brake);
delay(750);
claw.set_value(false);

}