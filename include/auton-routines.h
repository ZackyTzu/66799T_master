#pragma once
#include "Template/arm.h"

// Cascade target, in cascade motor degrees, for each scoring level. Every one
// of these is the previous target divided by 1.5, so auton now scores lower
// across the board (old values in the comments).
enum class ScoringLevel {
  LEVEL_0 = 0,      // fully retracted, unchanged by /1.5
  LEVEL_1 = 200,    // was 300
  LEVEL_opt = 533,  // was 800
  LEVEL_2 = 1333,   // was 2000
  LEVEL_3 = 1733,   // was 2600
  LEVEL_tpf = 2133, // was 3200
  LEVEL_4 = 2533    // was 3800
};

// Drives the cascade to `level`'s degree target and the arm to `arm_pos`
// (ArmPosition::DOWN = 0, POS_2 = 665, POS_1 = 1160 -- see arm.h), and blocks
// until both have actually arrived (or CASCADE_SCORE_TIMEOUT_MS elapses).
// `cascade_velocity` defaults to CASCADE_SCORE_VELOCITY; pass a higher value
// (e.g. 200, the cascade motors' max rpm) to move the cascade faster.
extern const int CASCADE_SCORE_VELOCITY;
void score(ScoringLevel level, ArmPosition arm_pos, int cascade_velocity = CASCADE_SCORE_VELOCITY);

void left();
void left2();
void right();
void right2();
void skill();
