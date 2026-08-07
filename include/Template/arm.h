#pragma once
#include "main.h"
using namespace pros;

// Arm preset positions, driven by a background task via its own PID.
// B -> DOWN, A -> POS_1, X -> POS_2 (see Drive::control_arcade in drive.cpp).
enum class ArmPosition {
    DOWN = 0,
    POS_1 = 1,
    POS_2 = 2,
    POS_3 = 3,
    // Rotated to just before the claw opens, if the arm was resting at POS_2
    // (~160 degrees) when A is pressed -- see Drive::control_arcade's A-button
    // handling in drive.cpp.
    CLAW_CLEAR = 4,
    // B's target instead of DOWN, if the arm was resting at POS_2 with the
    // claw closed -- holds a game piece just off the ground instead of
    // slamming it down while still gripped. See Drive::control_arcade's
    // B-button handling in drive.cpp.
    DOWN_HOLD = 5,
    // Where A sends the arm the rest of the way once it opens the claw from
    // DOWN_HOLD -- stops a bit short of true DOWN instead of continuing all
    // the way to the hard stop. See Drive::control_arcade's A-button
    // handling in drive.cpp.
    DOWN_HOLD_FINAL = 6,
    // Just off the bottom hard stop (~7.5 degrees) -- not bound to a button,
    // call arm_set_position(ArmPosition::POS_4) where you need it.
    POS_4 = 7,
    // Where arm_back() parks the arm in auton (~180 degrees), with the cascade
    // retracted to 0. Separate from POS_2/CLAW_CLEAR so it can be tuned
    // without moving the driver-control positions -- see arm_back() in
    // auton-routines.cpp.
    ARM_BACK = 8,
    // Same idea as ARM_BACK, a bit lower (~175 degrees) -- see arm_back2() in
    // auton-routines.cpp.
    ARM_BACK_2 = 9
};

extern ArmPosition arm_target;

// Target angles in ARM degrees, read straight off arm_rotation (the Rotation
// sensor on port 21, see robot-config.cpp). No gear-ratio conversion is
// involved anymore -- the sensor sits on the arm shaft, so what it reads IS
// the arm angle.
//
// Nothing zeroes the sensor at program start and there is no homing routine,
// so these are read straight off arm_rotation's existing zero -- see the note
// above ARM_DOWN_DEG in arm.cpp.
extern float ARM_DOWN_DEG;
extern float ARM_POS_1_DEG;
extern float ARM_POS_2_DEG;
extern float ARM_POS_3_DEG;
extern float ARM_POS_4_DEG;

// Target angle for ArmPosition::CLAW_CLEAR -- see the enum above.
extern float ARM_CLAW_CLEAR_DEG;

// Target angle for ArmPosition::DOWN_HOLD -- see the enum above.
extern float ARM_DOWN_HOLD_DEG;

// Target angle for ArmPosition::DOWN_HOLD_FINAL -- see the enum above.
extern float ARM_DOWN_HOLD_FINAL_DEG;

// Target angle for ArmPosition::ARM_BACK -- see the enum above.
extern float ARM_BACK_DEG;

// Target angle for ArmPosition::ARM_BACK_2 -- see the enum above.
extern float ARM_BACK_2_DEG;

// Soft travel limits in arm degrees. Every target is clamped into this range,
// so a bad preset can't drive the arm into its hard stop at full voltage.
// Set these to the arm's real measured travel.
extern float ARM_MIN_DEG;
extern float ARM_MAX_DEG;

// Set true if the sensor counts DOWN while the motor drives the arm UP. With
// this wrong the PID runs away from the target -- verify before tuning gains.
extern bool ARM_ROTATION_REVERSED;

// Arm position PID gains -- tune these to adjust how the arm reaches and
// holds its target angle. Error is in arm degrees. See arm_task() in arm.cpp.
extern float ARM_KP;
extern float ARM_KI;
extern float ARM_KD;
extern float ARM_STARTI;

// Constant gravity feedforward (volts), added to the PID output every loop.
// See ARM_KG in arm.cpp.
extern float ARM_KG;

// Separate, lower voltage cap applied whenever the PID output is driving the
// arm downward (e.g. heading to DOWN), so it descends gently instead of
// dropping at full speed. See arm_task() in arm.cpp.
extern const int ARM_DOWN_MAX_VOLTAGE;

// Voltage cap (either direction) applied while heading to CLAW_CLEAR or
// DOWN_HOLD, so the claw-clearance moves in Drive::control_arcade's A/B
// handling (drive.cpp) are gentler than a normal preset move.
extern const int ARM_SLOW_MAX_VOLTAGE;

// While non-zero, replaces ALL of the caps above (ARM_MAX_VOLTAGE,
// ARM_DOWN_MAX_VOLTAGE, ARM_SLOW_MAX_VOLTAGE) for as long as it is set, in
// both directions. Set it around a move that needs to run faster than the
// normal caps allow and clear it back to 0 afterwards -- see cascade_level_0()
// in auton-routines.cpp, which runs the arm at the full 127.
extern int arm_max_voltage_override;

// Minimum output forced while unsettled, to break static friction. If this
// is too high it overpowers KP near the settle boundary and causes a
// bang-bang oscillation right as the arm nears its target -- see arm.cpp.
extern float ARM_MIN_VOLTAGE;

// Max error (arm degrees) to be considered "arrived".
extern float ARM_SETTLE_ERROR_DEG;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target -- poll
// this to wait for the arm to actually reach its target. Forced false while
// the rotation sensor is unplugged, since position is unknown then.
extern bool arm_settled;

// True while arm_rotation is not reporting a valid position (unplugged or
// failed). The arm is held at 0 V for as long as this is set.
extern bool arm_sensor_ok;

extern float arm_level_1_rotate;

// --- vexdash live telemetry (streamed to the web dashboard) ---
extern float tele_arm_angle;   // current arm angle (deg)
extern float tele_arm_target;  // target arm angle (deg)
extern float tele_arm_error;   // target - current (deg)
extern float tele_arm_output;  // arm PID output (volts)

// Current arm angle in degrees from the rotation sensor. Returns the last
// valid reading if the sensor is unplugged.
float arm_get_position_deg();

// Target angle in arm degrees for a preset, already clamped to the soft limits.
float arm_target_degrees(ArmPosition pos);

void arm_set_position(ArmPosition pos);
void arm_task();
void start_arm_task();
