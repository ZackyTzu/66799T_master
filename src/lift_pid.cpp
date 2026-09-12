#include "main.h"
#include "lift_pid.h"
#include "tune.h"

#include <cmath>
#include <cstdint>

/*===========================================================================*/
/*  手臂的位置 PID                                                            */
/*                                                                           */
/*  這個檔案只做一件事：把 lift1 / lift2 推到某個角度，並且撐住不讓重力拉下來。*/
/*  用的是這個專案本來就有的 PID 類別（include/Template/PID.h，跟底盤同一套）， */
/*  沒有另外造一套——多出來的只有「抗重力前饋」那一項，而那一項是加在 PID 算完 */
/*  之後，不需要動 PID 類別本身。                                             */
/*                                                                           */
/*  原版：D:\VEX v5\AI專案\DPLIB\jar-template\src\JAR-Template\mech.cpp        */
/*  跟原版的差異只有三處（每處都在下面標了「差異」）：                        */
/*    1. 回授不是 motor_group 的編碼器，是 arm_rotation（這台車手臂上就有一顆  */
/*       旋轉感測器，control_arcade() 讀的也是它）。                          */
/*    2. 輸出走 pros::Motor::move()（-127~127），不是 VEXcode 的伏特。         */
/*    3. 沒有 tune_log_mech()：這台車的曲線走 vexdash::watch()，手臂位置已經   */
/*       由 main.cpp 的 watch_motor("lift1", lift1) 串上去了。                 */
/*===========================================================================*/

// 一次動作的硬上限。跟 src/tune.cpp 的 kMaxRunMs 是同一個數字，但**刻意各自
// 寫一份**：tune_stop_requested() 只在「有 kJob 在跑」時才會回 true，萬一以後
// 有人在 tune 以外的地方叫 move_to()，那條保險是不會生效的。這裡自己再數一次，
// 手臂就永遠停得下來。
static const std::uint32_t kLiftMaxRunMs = 10000;

bool LiftPID::installed() {
  // PROS 的 Rotation::get_position() 讀不到時回 PROS_ERR（INT32_MAX）。
  // 不檢查的話 position() 會回 2 千多萬度，PID 直接給滿輸出頂死機構。
  return arm_rotation.get_position() != PROS_ERR;
}

/**
 * 現在幾度。
 * ⚠ 這一行要跟 src/Template/drive.cpp 的 control_arcade() 裡的 arm_deg
 *   **逐字一樣**（同樣 /100.0、同樣取絕對值）。兩邊口徑一旦分岔，
 *   lift_top_deg / lift_bottom_deg 那組數字就會一邊對一邊錯。
 */
float LiftPID::position() {
  return (float)std::fabs(arm_rotation.get_position() / 100.0);
}

/**
 * 手臂現在跟水平差幾度。
 * deg_per_motor_deg 是 0（預設）時永遠回 0，抗重力就退化成固定輸出——
 * 這是刻意的，不是漏寫，見 lift_pid.h 的說明。
 */
float LiftPID::arm_angle_deg(float deg) {
  return (deg - horizontal_deg) * deg_per_motor_deg;
}

/**
 * 這個角度要給多少輸出才不會被重力拉下來。
 *     水平（0 度）   -> cos(0)  = 1  -> 給滿 kg
 *     立起來（90度） -> cos(90) = 0  -> 不用給
 * cos 本來就會處理過頭的情況：超過 90 度變負的，代表手臂倒到另一邊了，
 * 這時候要反過來撐，公式自己就會給負的，不用特別寫 if。
 */
float LiftPID::gravity_output(float deg) {
  return kg * cosf(to_rad(arm_angle_deg(deg)));
}

/**
 * 跑到某個角度，到位為止。
 *
 * 跟 chassis.drive_distance() 同一個形狀：一個 while 迴圈，每 10ms 算一次要給
 * 多少輸出。四個地方不一樣：
 *   1. 目標會先被夾在 min_position ~ max_position 之間（打錯數字的保險）。
 *   2. 輸出 = PID 算出來的 + 抗重力，然後才夾到 ±max_voltage。
 *   3. 到位之後不是馬上收工，而是再撐 hold_ms 毫秒——這幾百毫秒就是你在電腦
 *      曲線上看「放著到底會不會掉」的那一段，也是調 kg 的依據。
 *   4. 逾時（不是到位）就直接收工，不再撐 hold_ms：逾時多半是卡住或撞到死點，
 *      那種時候再全力頂快一秒等於在死點上悶燒馬達。
 */
void LiftPID::move_to(float target_position) {
  // 感測器沒接好就什麼都不做。
  if (!installed()) return;

  // 夾限：只准跑到安全範圍內。
  target = clamp(target_position, min_position, max_position);

  PID pid(target - position(), kp, ki, kd, starti, settle_error, settle_time, timeout);

  bool          holding   = false;  // 已經到位、正在撐住
  std::uint32_t hold_from = 0;      // 從幾點開始撐的
  std::uint32_t start_ms  = pros::millis();

  while (true) {
    float actual = position();
    float error  = target - actual;
    float grav   = gravity_output(actual);
    float output = clamp(pid.compute(error) + grav, -max_voltage, max_voltage);

    lift1.move(output);
    lift2.move(output);

    // 電腦上按了 STOP（tune 模式）、或這一次動作跑超過 10 秒，馬上收工。
    // 兩條保險都要，理由見上面 kLiftMaxRunMs 的說明。
    if (tune_stop_requested()) break;
    if (pros::millis() - start_ms > kLiftMaxRunMs) break;

    if (pid.is_settled()) {
      // ⚠ PID::is_settled() 有兩種情況都回 true（src/Template/PID.cpp:125-134）：
      //     (a) 誤差夠小、而且穩住夠久 -> 真的到位了
      //     (b) 跑超過 timeout        -> 時間到了，放棄
      //   (b) 代表手臂根本沒到位，多半是卡住。這種時候絕對不能再頂 hold_ms。
      if (timeout != 0 && pid.time_spent_running > timeout) break;

      if (!holding) {
        holding   = true;
        hold_from = pros::millis();
      }
      if (pros::millis() - hold_from > (std::uint32_t)hold_ms) break;
    }

    pros::delay(10);
  }
}

/**
 * 放開手臂。
 *
 * ⚠ 用的是馬達內建的電氣煞車（HOLD），**不是**軟體撐住。這跟 control_arcade()
 *   平常放開 L1/L2 之後的狀態是一樣的，所以 tune 模式退出時手臂的手感不會變。
 *   - 程式還在跑、但沒人叫 move_to()   -> 電氣煞車，會隨重力慢慢下垂。
 *   - 程式停掉 / Brain 關機 / 電池掉線 -> 馬達完全沒力，手臂會**直接掉下來**。
 *   真正的軟體撐住只在 move_to() 那個迴圈裡有效（含到位後的 hold_ms 那一段）。
 */
void LiftPID::stop() {
  lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift1.brake();
  lift2.brake();
}

/*===========================================================================*/
/*  這台車的手臂                                                              */
/*===========================================================================*/

LiftPID lift_pid;

void lift_pid_sync_limits() {
  // 上限沿用開迴路那組已經在用的 lift_top_deg（預設 270）：兩套控制共用同一個
  // 機械上限，調一邊不會忘了另一邊。下限維持 0——lift_bottom_deg（預設 5）是
  // 「已經到底了」的**判定門檻**，不是機械死點，拿來當 PID 的下限會讓手臂永遠
  // 降不到真正的最低點。
  lift_pid.max_position = chassis.lift_top_deg;
  lift_pid.min_position = 0;
}
