#include "main.h"

const int CASCADE_SCORE_VELOCITY = 100;   // move_absolute() speed for score(), out of 200 rpm
const int CASCADE_SCORE_SETTLE_DEG = 20;  // max cascade error to be considered "arrived"
const int CASCADE_SCORE_TIMEOUT_MS = 3000; // give up waiting and move on after this long
void score(ScoringLevel level, ArmPosition arm_pos){
  int cascade_target = (int)level;

  arm_set_position(arm_pos);
  cascade1.move_absolute(cascade_target, CASCADE_SCORE_VELOCITY);
  cascade2.move_absolute(cascade_target, CASCADE_SCORE_VELOCITY);

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
  delay(450);
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


void left(){
default_constants();
toggle.set_value(false);
chassis.drive_with_voltage(-67,-67);
delay(500);
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
chassis.drive_distance(14.75,false);
chassis.turn_to_angle(288.25);
chassis.drive_distance(12);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(100);
claw.set_value(false);
delay(100);
chassis.drive_distance(-4);
chassis.turn_to_angle(235);
chassis.drive_max_voltage = 107;
chassis.drive_distance(14.5,218.5);
chassis.turn_to_angle(220.5);
chassis.drive_max_voltage = 77;
chassis.drive_distance(4);
chassis.turn_to_angle(206);
delay(150);
claw.set_value(true);
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_2_task = Task(cascade_level_2_lift);
chassis.turn_to_angle(255);
chassis.drive_distance(-12.25);
chassis.turn_to_angle(308);
chassis.drive_distance(8.25);
Task claw_task = Task(wait_claw_open);
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
chassis.drive_distance(-5);
chassis.turn_to_angle(277.5);
chassis.wall_distance(Drive::WallSide::LEFT, -32.45, 282.5, 410, 0);
chassis.drive_stop(MotorBrake::brake);
Task cascade0_task = Task(cascade_level_0);
chassis.turn_to_angle(280);
chassis.drive_distance(6.5);
chassis.turn_to_angle(150);

default_constants();
chassis.drive_distance(18);
chassis.turn_to_angle(166);
chassis.drive_distance(4.25);
chassis.turn_to_angle(180);
delay(150);
claw.set_value(true);
chassis.drive_max_voltage = 117;
chassis.drive_distance(-1);
Task level_3_task = Task(cascade_level_3_lift);
chassis.turn_to_angle(155);
chassis.drive_distance(-15);
chassis.turn_to_angle(93);
chassis.drive_distance(7);
Task claw_task2 = Task(wait_claw_open);
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
chassis.drive_stop(MotorBrake::brake);

}

void right(){
default_constants();

}

void sawp(){
default_constants();
}