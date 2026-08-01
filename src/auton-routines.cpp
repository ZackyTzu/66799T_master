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
  delay(350);
  claw.set_value(false);
  return 0;
}

int wait_claw_open2(){
  delay(200);
  claw.set_value(false);
  return 0;
}

int cascade_level_3_lift(){
score(ScoringLevel::LEVEL_3, ArmPosition::DOWN);
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

void left(){
default_constants();
toggle.set_value(true);
chassis.drive_with_voltage(-57,-57);
delay(500);
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
chassis.drive_distance(15,false);
chassis.turn_to_angle(288);
chassis.drive_distance(12);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-5.5);
chassis.turn_to_angle(345);
chassis.drive_max_voltage = 107;
chassis.drive_distance(17.75);
chassis.drive_max_voltage = 20;
chassis.drive_distance(8.5);
claw.set_value(true);
Task level_2_task = Task(cascade_level_2_lift);
default_constants();
chassis.drive_distance(-1);
chassis.turn_max_voltage = 67;
chassis.turn_to_angle(206);
chassis.drive_distance(11.85);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(600);

default_constants();
chassis.drive_distance(-7.5);
Task cascade0_task = Task(cascade_level_0);
chassis.turn_to_angle(252.25);
chassis.drive_distance(21.5);
chassis.drive_max_voltage = 20;
chassis.drive_distance(7);
claw.set_value(true);
arm_set_position(ArmPosition::DOWN_HOLD);
Task level_tpf_task = Task(cascade_level_tpf);

chassis.turn_max_voltage = 67;
chassis.turn_to_angle(112);

default_constants();
chassis.drive_max_voltage = 100;
chassis.drive_distance(12.75);

delay(250);
arm_set_position(ArmPosition::DOWN);
Task claw_task2 = Task(wait_claw_open);
Task level_2_task2 = Task(cascade_level_2_lift);
chassis.drive_stop(MotorBrake::brake);
}


void right(){
default_constants();
toggle.set_value(true);
chassis.drive_with_voltage(-57,-57);
delay(500);
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
chassis.drive_distance(15,false);
chassis.turn_to_angle(288);
chassis.drive_distance(12);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-4);
chassis.turn_to_angle(235);
chassis.drive_max_voltage = 107;
chassis.drive_distance(14.25,218.5);
chassis.turn_to_angle(220,true);
chassis.drive_max_voltage = 77;
chassis.drive_timeout = 650;
chassis.drive_distance(4.75);
chassis.turn_to_angle(206);
delay(150);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_2_task = Task(cascade_level_2_lift);
chassis.turn_to_angle(255);
chassis.drive_distance(-11.5);
chassis.turn_to_angle(306.75);
chassis.drive_distance(8);
Task claw_task = Task(wait_claw_open);
Task cascade1_task = Task(cascade_level_opt);
delay(500);
chassis.drive_distance(-5);
chassis.turn_to_angle(277.5);
chassis.drive_settle_error = 5;
chassis.drive_timeout = 550;
chassis.wall_distance(Drive::WallSide::LEFT, -31, 281, 405, 40);
Task cascade0_task = Task(cascade_level_0);

default_constants();
chassis.drive_with_voltage(-40,-40);
delay(350);
chassis.drive_stop(MotorBrake::brake);
chassis.turn_to_angle(280);
chassis.drive_distance(6.5);
chassis.turn_to_angle(153.5);

default_constants();
chassis.drive_distance(17);
chassis.turn_to_angle(166,true);
chassis.drive_timeout = 750;
chassis.drive_distance(5);
chassis.turn_to_angle(180);
delay(150);
claw.set_value(true);

default_constants();
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_3_task = Task(cascade_level_3_lift);
chassis.turn_to_angle(155);
chassis.drive_distance(-15);
chassis.turn_to_angle(97);
chassis.drive_distance(7);
Task claw_task2 = Task(wait_claw_open2);
score(ScoringLevel::LEVEL_2, ArmPosition::DOWN);
chassis.drive_stop(MotorBrake::brake);
}

void left2(){
default_constants();

}

void right2(){
default_constants();

}