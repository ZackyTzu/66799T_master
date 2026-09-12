#pragma once

/**
 * tune — JAR Template 那一套「調參模式」搬到這台車（PROS 版）。
 *
 * 對照原版：D:\VEX v5\AI專案\DPLIB\jar-template
 *           src/JAR-Template/tune.cpp、include/JAR-Template/tune.h
 *
 * 做三件事（跟 JAR 一模一樣）：
 *   1. 把 chassis 的 PID 常數開放給電腦端 dashboard，拉滑桿就即時生效（不用重編譯）。
 *      名字與分組**逐字照抄 JAR**，所以網頁上的分組長得跟範例車一樣。
 *   2. Brain 螢幕 POSITION 分頁右上角一顆 TUNE 鈕：按一下進調參模式、再按一下退出。
 *      進了之後控制器那幾顆鍵就是「跑一次測試動作」的按鈕（配置表在 tune.cpp）。
 *   3. 電腦 dashboard 上五顆按鈕：Run drive test / Run turn test / Run swing test /
 *      Run lift test / Run STOP，對到 chassis.drive_distance() / turn_to_angle() /
 *      swing_to_angle() / lift_pid.move_to()（include/lift_pid.h）。
 *
 * ⚠ 調參模式**預設關著**，關著的時候整台車跟沒有這個檔案時一模一樣
 *   （opcontrol 走原本的 chassis.control_arcade()，機構邏輯一個字都沒動）。
 *   接上正式場地控制器（Field Control）時會自動關掉。
 */

// ---- 啟動 ---------------------------------------------------------------
// 在 initialize() 裡、**vexdash::init_* 之前**呼叫：登記可調參數與 dashboard 按鈕。
// （vexdash 的規矩是「先登記、再啟動」，見 vexdash_pros.h 的說明。）
void tune_register_dashboard();

// 在 initialize() 裡、vexdash::init_* 之後呼叫：把背景小任務叫起來。
// 呼叫幾次都一樣（內部是 function-local static），opcontrol 再叫一次也無害。
void tune_start();

// ---- 模式開關（Brain 螢幕那顆 TUNE 鈕用）--------------------------------
bool tune_mode_enabled();
void tune_mode_set(bool on);

// ---- 給 drive.cpp 的三個鉤子 --------------------------------------------
// 動作迴圈每一圈問一次「該停了嗎」。回 true 就 break。
// 沒有測試動作在跑的時候（＝學生自己的 auton）永遠回 false，所以自動程式不受影響。
bool tune_stop_requested();

// 現在有沒有測試動作在跑。控制迴圈用它判斷「這時候不要拿搖桿的值蓋掉測試動作」。
bool tune_is_running();

// 開車用的小任務有沒有真的活起來。false = 電腦上的測試按鈕不會有反應。
bool tune_task_started();

// ---- 調參模式專用的開車迴圈 ----------------------------------------------
// opcontrol 在調參模式下走這個，而不是 chassis.control_arcade()。
// 差別只有一個：機構（lift / toggle / roller）不吃搖桿按鍵，
// 因為那幾顆鍵在調參模式下已經是測試按鈕了（按 L1 不該同時抬手臂又跑測試）。
// 手臂是唯一的例外：方向鍵會叫 lift_pid 跑位置 PID 測試，那段期間這個迴圈
// 不會去 brake 手臂（否則 PID 的輸出每 10ms 就被蓋掉一次，手臂不會動）。
// 搖桿照樣可以開車——兩次測試之間本來就要把車開回起點。
// 這個函式會一直跑到調參模式關掉才返回。
void tune_drive_loop();

// ---- Brain 螢幕要印的按鍵對照表數值 --------------------------------------
// 這幾個值在電腦端 dashboard 的 "controller" 那一組隨時會被改，
// 螢幕直接印當下數值才不會對不起來。
struct TuneCtlValues {
  float drive_inch;   // L1 前進 / L2 後退 一次走幾吋
  float swing_deg;    // R1 往左 / R2 往右 一次 swing 幾度（相對現在方位）
  float turn_deg[4];  // X / Y / A / B 各轉到哪個絕對方位
  // 方向鍵四顆各把手臂送到哪個角度，照機構真實高低排（[0] 下鍵最低 ~ [3] 上鍵最高）。
  // 單位＝控制器螢幕上那個 "arm: xxx"，見 include/lift_pid.h 的口徑說明。
  float lift_deg[4];
};
TuneCtlValues tune_ctl_values();
