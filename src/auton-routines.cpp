#include "main.h"

const int CASCADE_SCORE_VELOCITY = 110;   // move_absolute() speed for score(), out of 200 rpm
const int CASCADE_SCORE_SETTLE_DEG = 20;  // max cascade error to be considered "arrived"
const int CASCADE_SCORE_TIMEOUT_MS = 3000; // give up waiting and move on after this long
void score(ScoringLevel level, ArmPosition arm_pos, int cascade_velocity){
  int cascade_target = (int)level;

  arm_set_position(arm_pos);
  cascade1.move_absolute(cascade_target, cascade_velocity);
  cascade2.move_absolute(cascade_target, cascade_velocity);

  int waited_ms = 0;
  while(!arm_settled || fabs(cascade1.get_position() - cascade_target) > CASCADE_SCORE_SETTLE_DEG){
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

int cascade_level_0(){
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
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

int cascade_level_1(){
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
return 0;
}

void left(){
default_constants();
toggle.set_value(true);
chassis.drive_with_voltage(-57,-57);
delay(450);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(15,false);
chassis.turn_to_angle(288);
chassis.drive_distance(11.65);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6.75);
chassis.turn_to_angle(343.5);
chassis.drive_max_voltage = 127;
chassis.drive_distance(19);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.85);
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
chassis.drive_distance(-10.5);
Task cascade0_task = Task(cascade_level_0);
chassis.turn_to_angle(254);
chassis.drive_distance(24);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.5);
delay(100);
claw.set_value(true);
delay(250);
arm_set_position(ArmPosition::DOWN_HOLD);
Task level_tpf_task = Task(cascade_level_tpf);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(116);

default_constants();
chassis.drive_max_voltage = 107;
chassis.drive_distance(11.75);
delay(250);
arm_set_position(ArmPosition::DOWN);
Task claw_task2 = Task(wait_claw_open);
Task level_2_task2 = Task(cascade_level_2_lift);
delay(1000);
chassis.drive_distance(-5);
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


void right(){
default_constants();
toggle.set_value(true);
chassis.drive_with_voltage(-57,-57);
delay(450);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(15,false);
chassis.turn_to_angle(288);
chassis.drive_distance(11.65);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6);
chassis.turn_to_angle(235);
chassis.drive_max_voltage = 107;
chassis.drive_distance(15.5);
chassis.turn_to_angle(220,true);
chassis.drive_max_voltage = 107;
chassis.drive_timeout = 600;
chassis.drive_settle_error = 2;
chassis.drive_distance(5.25);
chassis.turn_to_angle(209);
chassis.drive_timeout = 400;
chassis.drive_distance(3);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_2_task = Task(cascade_level_2_lift);
chassis.turn_to_angle(255);
chassis.drive_distance(-12);
chassis.turn_to_angle(306.75);
chassis.drive_distance(8.85);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(300);
chassis.drive_distance(-5);
chassis.turn_to_angle(277.5);
Task cascade0_task = Task(cascade_level_0);
chassis.drive_settle_error = 6.5;
chassis.drive_settle_time = 20;
chassis.drive_timeout = 550;
chassis.wall_distance(Drive::WallSide::LEFT, -30, 281, 405, 50);

default_constants();
chassis.drive_with_voltage(-40,-40);
delay(550);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_to_angle(281);
chassis.drive_distance(7.25);
chassis.turn_to_angle(151);

default_constants();
chassis.drive_distance(17);
chassis.turn_to_angle(164.5,true);
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

void left2(){
default_constants();
toggle.set_value(true);
chassis.drive_with_voltage(-57,-57);
delay(450);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(15,false);
chassis.turn_to_angle(72);
chassis.drive_distance(11.5);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-6.75);
chassis.turn_to_angle(13.5);
chassis.drive_max_voltage = 127;
chassis.drive_distance(19);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6.85);
delay(150);
claw.set_value(true);

default_constants();
Task level_2_task = Task(cascade_level_2_lift);
default_constants();
chassis.drive_distance(-1);
chassis.turn_max_voltage = 70;
chassis.turn_to_angle(154);
chassis.drive_distance(10.25);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(500);

default_constants();
chassis.drive_distance(-10.5);
Task cascade0_task = Task(cascade_level_0);
chassis.turn_to_angle(107.5);
chassis.drive_distance(24);
chassis.drive_max_voltage = 20;
chassis.drive_distance(6);
delay(100);
claw.set_value(true);
delay(150);
arm_set_position(ArmPosition::DOWN_HOLD);
Task level_tpf_task = Task(cascade_level_tpf);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(247);

default_constants();
chassis.drive_max_voltage = 107;
chassis.drive_distance(12.25);
delay(250);
arm_set_position(ArmPosition::DOWN);
Task claw_task2 = Task(wait_claw_open);
Task level_2_task2 = Task(cascade_level_2_lift);
delay(1000);
chassis.drive_distance(-5);
chassis.drive_stop(MotorBrake::brake);
// delay(600);

// default_constants();
// Task cascade0_task3 = Task(cascade_level_0);
// chassis.drive_distance(-7);
// arm_set_position(ArmPosition::DOWN);
// chassis.turn_to_angle(201.5);
// chassis.drive_distance(19);
// chassis.turn_to_angle(19,true);
// chassis.drive_max_voltage = 77;
// chassis.drive_timeout = 450;
// chassis.drive_distance(4.75);
// chassis.turn_to_angle(178);
// delay(150);
// claw.set_value(true);

// default_constants();
// chassis.drive_max_voltage = 117;
// chassis.drive_distance(-1);
// Task level_4_task = Task(cascade_level_4);
// chassis.turn_to_angle(208);
// chassis.drive_distance(-20);
// chassis.turn_max_voltage = 67;
// chassis.turn_to_angle(252.25);
// chassis.drive_max_voltage = 117;
// chassis.drive_distance(6.85);
// delay(100);
// arm_set_position(ArmPosition::DOWN);
// delay(100);
// claw.set_value(false);
// chassis.drive_max_voltage = 37;
// chassis.drive_distance(3);
// chassis.drive_stop(MotorBrake::brake);
}

void right2(){
default_constants();
toggle.set_value(true);
chassis.drive_with_voltage(-57,-57);
delay(450);
Task level_1_task = Task(cascade_level_1);
chassis.drive_distance(15,false);
chassis.turn_to_angle(72);
chassis.drive_distance(11.5);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-5);
Task level_0_task = Task(cascade_level_0);
chassis.turn_to_angle(124.5);
chassis.drive_max_voltage = 107;
chassis.drive_distance(14.25);
chassis.turn_to_angle(140.75,true);
chassis.drive_max_voltage = 107;
chassis.drive_timeout = 600;
chassis.drive_settle_error = 2;
chassis.drive_distance(5.25);
chassis.turn_to_angle(149.5);
chassis.drive_timeout = 300;
chassis.drive_distance(2.5);
claw.set_value(true);

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
chassis.turn_to_angle(82.5);
chassis.drive_settle_error = 6;
chassis.drive_settle_time = 20;
chassis.drive_timeout = 550;
chassis.wall_distance(Drive::WallSide::RIGHT, -30, 79, 405, 40);
Task cascade0_task = Task(cascade_level_0);

default_constants();
chassis.drive_with_voltage(-40,-40);
delay(550);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_to_angle(80);
chassis.drive_distance(7);
chassis.turn_to_angle(205);

default_constants();
chassis.drive_distance(15.5);
chassis.turn_to_angle(194,true);
chassis.drive_timeout = 600;
chassis.drive_settle_error = 2;
chassis.drive_distance(5.5);
chassis.turn_to_angle(181);
chassis.drive_timeout = 300;
chassis.drive_distance(2.5);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 127;
chassis.drive_distance(-1);
Task level_3_task = Task(cascade_level_3_lift);
chassis.turn_to_angle(205);
chassis.drive_distance(-14.75);
Task level_3_task2 = Task(cascade_level_3_lift_arm);
chassis.turn_to_angle(264.5);
chassis.drive_timeout = 500;
chassis.drive_distance(7);
Task claw_task2 = Task(wait_claw_open2);
Task level_3_task3 = Task(cascade_level_3_lift);
chassis.drive_distance(-2);
chassis.drive_stop(MotorBrake::brake);
}