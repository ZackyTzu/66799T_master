#include "main.h"

// ---- Routines (selected on the dashboard's Auton Select tab) ----
//
// ⚠ 2026-09-13：每條路線開頭原本都有一行 default_constants()，已經全部拿掉。
//   留著的話，在 dashboard 上調好的 PID 永遠進不了 auton——調參跑的是滑桿上的
//   值、auton 跑的是 robot-config.cpp 裡硬寫的值，兩組根本不是同一份，調再久
//   也對不起來。常數現在只在 initialize() 載入一次（src/main.cpp:8）。
//   要回到出廠值：重開機，或自己在路線裡明寫 default_constants()。

void left(){
  chassis.drive_stop(MotorBrake::brake);
}

void left2(){
  chassis.drive_stop(MotorBrake::brake);
}

void right(){
  chassis.drive_stop(MotorBrake::brake);
}

void right2(){
  chassis.drive_stop(MotorBrake::brake);
}

void skill(){
  chassis.drive_stop(MotorBrake::brake);
}

