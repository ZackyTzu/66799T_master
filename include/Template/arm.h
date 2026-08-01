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
    DOWN_HOLD = 5
};

extern ArmPosition arm_target;

// Target angles in ARM degrees, read straight off arm_rotation (the Rotation
// sensor on port 21, see robot-config.cpp). No gear-ratio conversion is
// involved anymore -- the sensor sits on the arm shaft, so what it reads IS
// the arm angle.
//
// ARM_DOWN_DEG is 0: arm_task() zeroes the sensor at program start, so 0 is
// wherever the arm is resting when the program boots.
extern float ARM_DOWN_DEG;
extern float ARM_POS_1_DEG;
extern float ARM_POS_2_DEG;
extern float ARM_POS_3_DEG;

// Target angle for ArmPosition::CLAW_CLEAR -- see the enum above.
extern float ARM_CLAW_CLEAR_DEG;

// Target angle for ArmPosition::DOWN_HOLD -- see the enum above.
extern float ARM_DOWN_HOLD_DEG;

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
