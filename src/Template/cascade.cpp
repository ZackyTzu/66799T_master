#include "main.h"

float cascade_target = 0;

const float CASCADE_MIN_DEG = 0;
float CASCADE_MAX_DEG = 3875; // was CASCADE_EXTEND_LIMIT_DEG in drive.cpp before the cascade got its own PID

// Starting point only -- these need tuning on the robot, same as ARM_KP/KI/KD.
float CASCADE_KP = 4;
float CASCADE_KI = 0;
float CASCADE_KD = 0;
float CASCADE_STARTI = 0;

const int CASCADE_MAX_VOLTAGE = 127;
const int CASCADE_DOWN_MAX_VOLTAGE = 100;

float CASCADE_SETTLE_ERROR_DEG = 20; // was CASCADE_SETTLE_ERROR_DEG in drive.cpp

bool cascade_settled = false;
bool cascade_pid_active = false;

float tele_cascade_position = 0;
float tele_cascade_target = 0;
float tele_cascade_error = 0;
float tele_cascade_output = 0;

void cascade_set_target(float target_deg){
  cascade_target = clamp(target_deg, CASCADE_MIN_DEG, CASCADE_MAX_DEG);
  cascade_pid_active = true;
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

  PID cascadePID(0, CASCADE_KP, CASCADE_KI, CASCADE_KD, CASCADE_STARTI);

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
      delay(10);
      continue;
    }

    // Re-read gains every loop (instead of only at task start) so dashboard
    // slider changes to CASCADE_KP/KI/KD take effect immediately.
    cascadePID.kp = CASCADE_KP;
    cascadePID.ki = CASCADE_KI;
    cascadePID.kd = CASCADE_KD;
    cascadePID.starti = CASCADE_STARTI;

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

    cascade_settled = fabs(error) < CASCADE_SETTLE_ERROR_DEG;
    delay(10);
  }
}

void start_cascade_task(){
  static Task cascade_bg_task(cascade_task);
}
