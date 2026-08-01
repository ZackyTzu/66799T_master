#pragma once
#include "main.h"
using namespace pros;

// Cascade lift target, in cascade motor degrees (read straight off cascade1's
// encoder -- cascade2 is mechanically linked and driven with the same
// voltage). Driven by a background task running its own PID -- see
// cascade_task() in cascade.cpp. 0 is the physical bottom hard stop, found by
// cascade_limit (see robot-config.cpp) and re-zeroed every time it's pressed.
extern float cascade_target;

// Cascade never allowed to travel past this (cascade motor degrees, measured
// from the hard stop at 0). Targets passed to cascade_set_target() are
// clamped into this range, same idea as ARM_MIN_DEG/ARM_MAX_DEG in arm.h.
extern const float CASCADE_MIN_DEG;
extern float CASCADE_MAX_DEG;

// The cascade runs its own PID (see PID.h/PID.cpp, same class the arm/drive/
// turn PID uses) against cascade1's encoder -- tune these directly.
extern float CASCADE_KP;
extern float CASCADE_KI;
extern float CASCADE_KD;
extern float CASCADE_STARTI; // max error (cascade degrees) before the I term starts accumulating

extern const int CASCADE_MAX_VOLTAGE;      // out of 127, clamps the PID output while raising
extern const int CASCADE_DOWN_MAX_VOLTAGE; // out of 127, clamps output while lowering, so it descends gently

// Max error (cascade degrees) to be considered "arrived" -- see cascade_settled.
extern float CASCADE_SETTLE_ERROR_DEG;

// True once the cascade is within CASCADE_SETTLE_ERROR_DEG of cascade_target.
extern bool cascade_settled;

// True while the background PID task should be driving cascade1/cascade2.
// Driver's manual L1/L2 jog in Drive::control_arcade() sets this false while
// held (and drives the motors itself, bypassing the PID), then
// cascade_set_target() sets it true again to hand control back for a preset.
extern bool cascade_pid_active;

// --- vexdash live telemetry (streamed to the web dashboard) ---
extern float tele_cascade_position; // current cascade position (deg)
extern float tele_cascade_target;   // target cascade position (deg)
extern float tele_cascade_error;    // target - current (deg)
extern float tele_cascade_output;   // cascade PID output (volts)

// Sets a new target (clamped to CASCADE_MIN_DEG/CASCADE_MAX_DEG) and hands
// motor control back to the background PID task.
void cascade_set_target(float target_deg);

void cascade_task();
void start_cascade_task();
