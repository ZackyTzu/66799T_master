#include "main.h"

float cascade_target = 0;

const float CASCADE_MIN_DEG = 0;
float CASCADE_MAX_DEG = 3800; // was CASCADE_EXTEND_LIMIT_DEG in drive.cpp before the cascade got its own PID

// Starting point only -- these need tuning on the robot, same as ARM_KP/KI/KD.
float CASCADE_KP = 4;
float CASCADE_KI = 0;
float CASCADE_KD = 0;
float CASCADE_STARTI = 0;

const int CASCADE_MAX_VOLTAGE = 127;
const int CASCADE_DOWN_MAX_VOLTAGE = 107;

// Fed into cascadePID's settle_error/settle_time every loop, so the cascade
// settles through PID::is_settled() rather than an instantaneous |error| test
// -- see cascade.h. This is the single settle knob for the cascade; the preset
// sequences in drive.cpp wait on it too.
float CASCADE_SETTLE_ERROR = 25;   // cascade degrees
float CASCADE_SETTLE_TIME_MS = 0;  // 0 = settled as soon as error enters the band

bool cascade_settled = false;
bool cascade_pid_active = false;

// Bumped by every cascade_set_target() call -- see arm_target_generation in
// arm.cpp for why the counter exists.
static int cascade_target_generation = 0;

float tele_cascade_position = 0;
float tele_cascade_target = 0;
float tele_cascade_error = 0;
float tele_cascade_output = 0;

// Clears cascade_settled before returning, so a caller polling it on the next
// line can't read the previous target's result -- see arm_set_position().
void cascade_set_target(float target_deg){
  cascade_target = clamp(target_deg, CASCADE_MIN_DEG, CASCADE_MAX_DEG);
  cascade_pid_active = true;
  cascade_settled = false;
  cascade_target_generation++;
}

void cascade_task(){
  // Default brake mode is COAST, which would let the cascade sag under
  // gravity between PID updates instead of holding position.
  cascade1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cascade2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Unlike the arm, the cascade doesn't need a drive-to-stall homing routine
  // -- it's assumed to be resting on cascade_limit (its physical bottom hard
  // stop) whenever the program starts, so tare now. cascade_limit re-zeros
  // this again below the first time it's actually pressed, which also
  // corrects any drift picked up over a match.
  cascade1.tare_position();
  cascade2.tare_position();

  // settle_error/settle_time come from the globals every loop below; timeout
  // stays 0 (= never), same reason as armPID -- this runs for the whole match.
  PID cascadePID(0, CASCADE_KP, CASCADE_KI, CASCADE_KD, CASCADE_STARTI);

  int settled_for_generation = -1;

  while(true){
    // Cascade limit switch: reads 1 when pressed. Every time it's triggered,
    // re-zero both cascade encoders so the physical hard stop is always
    // "0 degrees" -- corrects any encoder drift picked up over the match.
    if(cascade_limit.get_value() == 1){
      cascade1.tare_position();
      cascade2.tare_position();
    }

    // Manual L1/L2 jog (see Drive::control_arcade) drives the motors itself
    // and clears this flag while held, so the PID doesn't fight the driver.
    if(!cascade_pid_active){
      tele_cascade_output = 0;
      // The driver is placing the cascade by hand, so "settled on
      // cascade_target" is meaningless -- and time banked in the band while the
      // PID wasn't driving shouldn't count toward the next move's settle time.
      cascade_settled = false;
      cascadePID.time_spent_settled = 0;
      delay(10);
      continue;
    }

    // Re-read gains every loop (instead of only at task start) so dashboard
    // slider changes to CASCADE_KP/KI/KD take effect immediately.
    cascadePID.kp = CASCADE_KP;
    cascadePID.ki = CASCADE_KI;
    cascadePID.kd = CASCADE_KD;
    cascadePID.starti = CASCADE_STARTI;
    cascadePID.settle_error = CASCADE_SETTLE_ERROR;
    cascadePID.settle_time = CASCADE_SETTLE_TIME_MS;

    int generation = cascade_target_generation;
    if(generation != settled_for_generation){
      cascadePID.time_spent_settled = 0;
      cascadePID.accumulated_error = 0;
      settled_for_generation = generation;
    }

    float position = cascade1.get_position();
    float error = cascade_target - position;

    tele_cascade_position = position;
    tele_cascade_target = cascade_target;
    tele_cascade_error = error;

    float output = cascadePID.compute(error);

    int max_voltage = output < 0 ? CASCADE_DOWN_MAX_VOLTAGE : CASCADE_MAX_VOLTAGE;
    output = clamp(output, (float)-max_voltage, (float)max_voltage);

    cascade1.move(output);
    cascade2.move(output);
    tele_cascade_output = output;

    // Same guard as arm_task(): don't stamp this loop's result onto a target
    // that cascade_set_target() replaced while we were computing.
    if(generation == cascade_target_generation){
      cascade_settled = cascadePID.is_settled();
    }
    delay(10);
  }
}

void start_cascade_task(){
  static Task cascade_bg_task(cascade_task);
}
