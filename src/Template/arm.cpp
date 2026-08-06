#include "main.h"

ArmPosition arm_target = ArmPosition::DOWN;

// Target angles in ARM degrees, straight from arm_rotation (port 21).
//
// To re-measure any of these:
//   1. Build and run, then press any arm button once so it homes (see
//      arm_task()) -- 0 becomes the arm's real bottom hard stop, not
//      wherever it happened to be sitting when the program started.
//   2. Move the arm by hand (or with the buttons) to the position you want.
//   3. Read "arm_angle" on the vexdash Graph panel and put that number here.
// Or drag ARM_*_DEG on the dashboard Config panel and watch the arm move --
// the values write back live, so you can find them without rebuilding.
//
// DOWN is 0, and 0 is a FIXED physical position -- the arm's bottom hard
// stop -- not "wherever the arm was at boot".
float ARM_DOWN_DEG = 1;
float ARM_POS_1_DEG = 287;
float ARM_POS_2_DEG = 157.5;   // LEFT sequence's final position, after the cascade is back at 0
float ARM_POS_3_DEG = 265;  // LEFT sequence's raised position, before coming back to POS_2
float ARM_POS_4_DEG = 7.5;  // just off the bottom hard stop -- see ArmPosition::POS_4 in arm.h
float ARM_CLAW_CLEAR_DEG = 180; // rotated to before the claw opens, if the arm was resting at POS_2 (~160) -- see Drive::control_arcade's A-button handling in drive.cpp
float ARM_DOWN_HOLD_DEG = 28.5; // B's target instead of DOWN, if the arm was at POS_2 with the claw closed -- see Drive::control_arcade's B-button handling in drive.cpp
float ARM_DOWN_HOLD_FINAL_DEG = 10; // where A continues to from DOWN_HOLD once the claw opens -- see Drive::control_arcade's A-button handling in drive.cpp
float ARM_BACK_DEG = 180;   // where arm_back() parks the arm in auton -- see arm_back() in auton-routines.cpp
float ARM_BACK_2_DEG = 175; // where arm_back2() parks the arm in auton -- see arm_back2() in auton-routines.cpp

// Soft travel limits in arm degrees. Targets are clamped here so a bad preset
// stalls the motor against nothing instead of slamming the hard stop.
// Set ARM_MAX_DEG to the arm's real measured travel.
float ARM_MIN_DEG = 0;
float ARM_MAX_DEG = 287;

// The rotation sensor must count UP when the motor drives the arm UP. If the
// arm runs away from its target instead of settling on it, flip this first --
// it's the usual cause, not the gains.
bool ARM_ROTATION_REVERSED = false;

// The arm runs its own PID (see PID.h/PID.cpp, same class the drive/turn PID
// uses) against the rotation sensor -- tune these directly.
//

float ARM_KP = 2;
float ARM_KI = 0;
float ARM_KD = 0.3; // damps the overshoot/oscillation that KP alone produces near the target
float ARM_STARTI = 10; // max error (arm degrees) before the I term starts accumulating

// Constant gravity feedforward, added to the PID output every loop (not just
// while unsettled) so the arm doesn't rely on KP alone to hold itself up.
//
// To measure it: get the arm settled (not oscillating) at the problem angle,
// then read the steady-state "arm_output" value on the vexdash Graph panel --
// that's roughly how many volts it takes just to hold there against gravity.
// Set ARM_KG to that. Do this at your highest-torque position (looks like
// POS_2/160 here); a single constant won't be exactly right at every angle,
// but it removes most of the load KP+KD would otherwise have to fight.
float ARM_KG = 0;

const int ARM_MAX_VOLTAGE = 97; // out of 127, clamps the PID output
const int ARM_DOWN_MAX_VOLTAGE = 77; // out of 127, clamps output while descending so the arm goes down slower
const int ARM_SLOW_MAX_VOLTAGE = 77; // out of 127, clamps output (either direction) while heading to CLAW_CLEAR or DOWN_HOLD, so those claw-sequence moves are gentler than a normal preset move

// 0 = use the caps above. Anything else replaces all three, in both
// directions, for as long as it's set -- see arm.h and cascade_level_0() in
// auton-routines.cpp.
int arm_max_voltage_override = 0;

// Whenever the arm hasn't settled yet, its output is forced to at least this
// much (in the direction of error), even if KP*error alone would be smaller.
// Without this, a small-but-not-settled error near a target (most visibly at
// DOWN/0) produces too little voltage to break static friction, and the arm
// just stalls short instead of stalling at 0 like it should.
//
// Keep this as small as it can be and still break static friction. Too high
// and it overpowers KP right at the settle boundary: the arm gets slammed
// toward the target at full ARM_MIN_VOLTAGE the instant error crosses
// ARM_SETTLE_ERROR_DEG, overshoots past it, error flips sign, and it gets
// slammed back the other way -- a bang-bang limit cycle that looks like wild
// oscillation and never settles. That's almost certainly what a violent
// oscillation right as the arm nears a target is -- lower this first before
// touching KP/KD. Live-tunable on the dashboard (arm/pid, "minV").
float ARM_MIN_VOLTAGE = 25; // out of 127

// Max error (arm degrees) to be considered "arrived" -- see arm_settled below.
// The old 20 motor degrees was ~6.7 arm degrees; this is a bit tighter. Loosen
// it if preset sequences start hitting their PRESET_STEP_TIMEOUT_MS.
float ARM_SETTLE_ERROR_DEG = 5.75;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target. Lets other
// code (e.g. Drive::control_arcade) wait for the arm to actually get there
// before doing something that depends on it, instead of guessing a delay.
bool arm_settled = false;

bool arm_sensor_ok = false;

// True once the arm has driven down and found its real physical zero (see
// arm_task()'s homing step). Nothing drives the arm until this is true, so
// the arm never moves on its own just because the program started -- it only
// homes the first time a button actually asks the arm to go somewhere.
bool arm_homed = false;
static bool arm_home_requested = false;

float tele_arm_angle = 0;
float tele_arm_target = 0;
float tele_arm_error = 0;
float tele_arm_output = 0;

// Last known-good angle, so a momentary sensor dropout doesn't read as 0 (which
// would look like a huge error and slam the arm).
static float arm_last_good_deg = 0;

void arm_set_position(ArmPosition pos){
  // Whichever button/routine asks the arm to go somewhere first is what
  // triggers homing -- not the program starting. See arm_task().
  if(!arm_homed){
    arm_home_requested = true;
  }
  arm_target = pos;
}

// Rotation::get_position() returns centidegrees, and PROS_ERR when the sensor
// isn't reporting. Updates arm_sensor_ok as a side effect.
float arm_get_position_deg(){
  std::int32_t centideg = arm_rotation.get_position();

  if(centideg == PROS_ERR){
    arm_sensor_ok = false;
    return arm_last_good_deg;
  }

  arm_sensor_ok = true;
  arm_last_good_deg = centideg / 100.0;
  return arm_last_good_deg;
}

float arm_target_degrees(ArmPosition pos){
  float arm_deg;
  switch(pos){
    case ArmPosition::DOWN:  arm_deg = ARM_DOWN_DEG;  break;
    case ArmPosition::POS_1: arm_deg = ARM_POS_1_DEG; break;
    case ArmPosition::POS_2: arm_deg = ARM_POS_2_DEG; break;
    case ArmPosition::POS_3: arm_deg = ARM_POS_3_DEG; break;
    case ArmPosition::POS_4: arm_deg = ARM_POS_4_DEG; break;
    case ArmPosition::CLAW_CLEAR: arm_deg = ARM_CLAW_CLEAR_DEG; break;
    case ArmPosition::DOWN_HOLD: arm_deg = ARM_DOWN_HOLD_DEG; break;
    case ArmPosition::DOWN_HOLD_FINAL: arm_deg = ARM_DOWN_HOLD_FINAL_DEG; break;
    case ArmPosition::ARM_BACK: arm_deg = ARM_BACK_DEG; break;
    case ArmPosition::ARM_BACK_2: arm_deg = ARM_BACK_2_DEG; break;
    default:                 arm_deg = ARM_DOWN_DEG;  break;
  }
  return clamp(arm_deg, ARM_MIN_DEG, ARM_MAX_DEG);
}

// Homing voltage/timing for arm_task()'s startup homing routine below.
const int ARM_HOME_VOLTAGE = -40;     // out of 127, negative = descending
const int ARM_HOME_STALL_RPM = 2;     // |velocity| below this counts as "not moving"
const int ARM_HOME_STALL_MS = 200;    // how long it must stay stalled to count as "hit the hard stop"
const int ARM_HOME_TIMEOUT_MS = 2000; // give up and zero wherever it ends up, so a jam can't hang boot forever

void arm_task(){
  // Default brake mode is COAST, which would let the arm sag under gravity
  // between PID updates instead of holding position.
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  arm_rotation.set_reversed(ARM_ROTATION_REVERSED);
  arm_rotation.set_data_rate(5); // ms, so the 10ms loop always has a fresh sample

  // The motor encoder no longer drives the PID, but the MOTORS tab of the V5
  // dashboard shows it, so keep it zeroed at the same moment as arm_rotation
  // (inside the homing step below).
  arm.tare_position();

  PID armPID(0, ARM_KP, ARM_KI, ARM_KD, ARM_STARTI);

  while(true){
    // Nothing below here drives the arm until it's been homed at least once
    // -- that's what keeps the arm from moving on its own just because the
    // program started. arm_set_position() sets arm_home_requested the first
    // time ANY position (DOWN, POS_1, ...) is actually asked for, so whatever
    // button is pressed first triggers this.
    //
    // The arm can start at any angle -- however it happened to be sitting
    // when the program booted -- so just taring the sensor at boot (the old
    // approach) would make "0" mean "wherever it randomly was," not the real
    // hard stop. That's why DOWN did nothing the first time it was pressed:
    // target and position were both "0" already, by definition, with zero
    // error. Driving down at a fixed voltage until the arm actually stalls
    // against its hard stop, THEN zeroing, anchors 0 to a real position no
    // matter where the arm started.
    if(arm_home_requested){
      arm.move(ARM_HOME_VOLTAGE);
      int stalled_ms = 0;
      int homing_ms = 0;
      while(stalled_ms < ARM_HOME_STALL_MS && homing_ms < ARM_HOME_TIMEOUT_MS){
        delay(10);
        homing_ms += 10;
        stalled_ms = (fabs(arm.get_actual_velocity()) < ARM_HOME_STALL_RPM) ? stalled_ms + 10 : 0;
      }
      arm.move(0);
      arm_rotation.reset_position(); // the true hard stop, just reached above, becomes 0
      arm.tare_position();
      arm_home_requested = false;
      arm_homed = true;
    }

    if(!arm_homed){
      arm.move(0);
      tele_arm_output = 0;
      arm_settled = false;
      delay(10);
      continue;
    }

    float target = arm_target_degrees(arm_target);
    float position = arm_get_position_deg();
    float error = target - position;

    tele_arm_angle = position;
    tele_arm_target = target;
    tele_arm_error = error;

    // With no trustworthy position there's no safe direction to drive, so stop
    // and let the brake mode hold the arm where it is.
    if(!arm_sensor_ok){
      arm.move(0);
      tele_arm_output = 0;
      arm_settled = false;
      delay(10);
      continue;
    }

    float output = armPID.compute(error) + ARM_KG;

    bool settled = fabs(error) < ARM_SETTLE_ERROR_DEG;
    if(!settled && fabs(output) < ARM_MIN_VOLTAGE){
      output = error > 0 ? ARM_MIN_VOLTAGE : -ARM_MIN_VOLTAGE;
    }

    bool slow_target = (arm_target == ArmPosition::CLAW_CLEAR || arm_target == ArmPosition::DOWN_HOLD);
    int max_voltage = slow_target ? ARM_SLOW_MAX_VOLTAGE : (output < 0 ? ARM_DOWN_MAX_VOLTAGE : ARM_MAX_VOLTAGE);
    if(arm_max_voltage_override > 0) max_voltage = arm_max_voltage_override;
    output = clamp(output, (float)-max_voltage, (float)max_voltage);

    arm.move(output);
    tele_arm_output = output;
    arm_settled = settled;
    delay(10);
  }
}

void start_arm_task(){
  static Task arm_bg_task(arm_task);
}
