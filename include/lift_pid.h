#pragma once

/**
 * lift_pid — 手臂（lift）的「位置 PID」：叫它抬到某個角度，就停在那裡。
 *
 * 這是 JAR Template 的 Mech 類別移植過來的：
 *   原版 D:\VEX v5\AI專案\DPLIB\jar-template
 *        include/JAR-Template/mech.h、src/JAR-Template/mech.cpp（class Mech，d4b）
 *
 * 為什麼要有這個檔：
 *   這台車原本的 lift 是**開迴路**——按著 L1 給固定電壓往上、放開就靠
 *   lift_hold_ff 撐住（src/Template/drive.cpp 的 Drive::control_arcade()）。
 *   開迴路沒有「目標角度」這回事，所以教練說的「調 lift 的 PID」在原本的
 *   程式裡是無從調起的。這個檔案補的就是那個缺的東西：位置回授 + PID。
 *
 * 比純 PID 多做一件事：**抗重力前饋**（下面的 kg）。
 *   純 PID 到了目標後誤差=0、輸出也=0，手臂就被重力拉下去，掉了才產生誤差、
 *   才又推上來——會一直上上下下抖。加一個「本來就要一直給」的輸出就不會。
 *   這台車其實已經有這個數字了：control_arcade() 的 lift_hold_ff（預設 12）
 *   就是「手放開不會掉」的最小輸出，所以 kg 的預設值直接沿用 12。
 *
 * 【單位】角度＝ arm_rotation 的度數，取**絕對值**：
 *        fabs(arm_rotation.get_position() / 100.0)
 *   跟 control_arcade() 裡的 arm_deg 是**同一個口徑**（同一行算式），
 *   所以 lift_top_deg / lift_bottom_deg 那組數字兩邊通用，螢幕上印的
 *   "arm: xxx" 也可以直接拿來當目標值抄。
 *   取絕對值的理由跟 control_arcade() 一樣：感測器往哪邊數要看怎麼裝的，
 *   讀到負值會讓所有比大小的判斷整組失效。
 *
 * 【輸出】lift1.move() / lift2.move()，單位是 **0~127**（不是 JAR 的 0~12 伏特）。
 *   PROS 的 Motor::move() 吃的就是 -127~127，這台車 control_arcade() 填的
 *   lift_up_volt=127 / lift_hold_ff=12 也都是這個尺標。照 JAR 夾成 12 的話
 *   手臂大概只會抖一下。成員名字保留 JAR 的 max_voltage 是為了兩邊對讀方便
 *   （src/tune.cpp 的 drive_max_voltage 也是同樣的處理，見該檔檔頭差異 2）。
 *
 * 【本輪只在 tune 模式用】
 *   平常駕駛（TUNE 關著）走的還是 chassis.control_arcade()，L1/L2 的行為
 *   一個字都沒有動。這個類別只被 src/tune.cpp 的背景小任務叫到。
 *
 * ⚠ 這個類別**不會**在沒有人叫 move_to() 的時候幫你撐住手臂。
 *   stop() 用的是馬達內建的電氣煞車（HOLD），程式停掉/關機就完全沒力，
 *   手臂會直接掉下來。測試前先把手臂下方清空。
 */

class LiftPID {
public:
  /*-------------------------------------------------------------------------*/
  /*  PID 常數（電腦 dashboard 上拉滑桿就是在改這幾個）                        */
  /*  誤差單位是「手臂度數」，全行程大約 0~270，所以 kp 是個位數。             */
  /*-------------------------------------------------------------------------*/
  float kp          = 1.0f;    // 差 127 度給滿輸出 -> 1.0 是很溫和的起點
  float ki          = 0.0f;
  float kd          = 3.0f;
  float starti      = 0.0f;
  float max_voltage = 100.0f;  // 0~127（不是伏特，見檔頭）

  /*-------------------------------------------------------------------------*/
  /*  抗重力前饋：     輸出 = kg * cos(手臂跟水平差幾度)                       */
  /*                                                                          */
  /*  kg = 「手臂剛好伸成水平時，要給多少輸出它才不會掉」。水平是最重的姿勢，  */
  /*       所以 kg 就是最大的那個抗重力輸出。預設 12 ＝ 現行 lift_hold_ff。    */
  /*  cos(...) = 手臂立起來之後力臂變短，需要的力也跟著變小。                  */
  /*                                                                          */
  /*  【重要】deg_per_motor_deg 設成 0（預設）時角度永遠算成 0、cos(0)=1，     */
  /*  整條公式退化成「一直給固定的 kg」——這就是現在 control_arcade() 在做的   */
  /*  事，最簡單也最好懂。等固定值調不好（低的時候太用力、高的時候不夠力）     */
  /*  再去量 horizontal_deg 跟 deg_per_motor_deg，開啟依角度變化的版本。       */
  /*-------------------------------------------------------------------------*/
  // 手臂水平時 arm_rotation 讀到幾度。把手臂擺成水平、看控制器螢幕的 arm: 抄下來。
  float horizontal_deg     = 0.0f;
  // 感測器 1 度 = 手臂抬起來幾度。0 = 不管角度、固定給 kg（預設）。
  // 感測器直接裝在手臂軸上的話這個值是 1。
  float deg_per_motor_deg  = 0.0f;
  // 水平時撐住手臂要多少輸出（0~127）。預設 12 = chassis.lift_hold_ff。
  float kg                 = 12.0f;

  /*-------------------------------------------------------------------------*/
  /*  什麼時候算「到了」、最多跑多久                                          */
  /*-------------------------------------------------------------------------*/
  float settle_error = 3.0f;     // 差幾度以內算到了
  float settle_time  = 200.0f;   // 要連續穩住幾毫秒才算到了
  float timeout      = 3000.0f;  // 最多跑幾毫秒就放棄（不要設 0，0 = 永不逾時）
  float hold_ms      = 800.0f;   // 到位後再撐住幾毫秒，好在電腦曲線上看它掉不掉

  /*-------------------------------------------------------------------------*/
  /*  安全範圍：只准跑到這兩個角度之間                                        */
  /*  打錯一個 0（想打 27 打成 270）不會讓手臂去撞機械死點硬卡住。             */
  /*  預設值沿用 control_arcade() 那組：底=0、頂=chassis.lift_top_deg（270）。 */
  /*  真正對齊是在 lift_pid_sync_limits()，initialize() 裡叫一次。            */
  /*-------------------------------------------------------------------------*/
  float min_position = 0.0f;
  float max_position = 270.0f;

  // 上一次叫它去哪（給螢幕/曲線看的，不要自己另外再存一份）。
  float target = 0.0f;

  // 感測器接好了沒。沒接好的話 move_to() 什麼都不做——硬跑的話角度會讀到
  // PROS_ERR，PID 會一路給滿輸出把手臂頂死在死點上。
  bool installed();

  // 現在幾度（跟 control_arcade() 的 arm_deg 同一條算式）。
  float position();

  // 手臂現在跟水平差幾度（deg_per_motor_deg 是 0 的話永遠回 0）。
  float arm_angle_deg(float deg);
  // 這個角度的抗重力輸出要給多少（0~127 尺標）。
  float gravity_output(float deg);

  // 跑到某個角度。**會卡住**直到：到位並撐滿 hold_ms、或超過 timeout、
  // 或電腦上按了 STOP、或超過 10 秒總上限。
  //
  // ⚠ 所以這個函式只准在 src/tune.cpp 的 tune_job_task 那個背景小任務裡叫，
  //   不要在 vexdash 的 callback 或控制器任務裡直接叫（會把那條線卡住）。
  //   dashboard 的按鈕與遙控器的方向鍵都是排一張 kJobLift 工作單，
  //   由那個小任務去跑——跟 drive/turn/swing 測試完全一樣的流程。
  void move_to(float target_position);

  // 放開。回到馬達內建的電氣煞車（HOLD），跟 control_arcade() 平常的狀態一致。
  void stop();
};

// 這台車的手臂。定義在 src/lift_pid.cpp 最下面。
extern LiftPID lift_pid;

// 把安全夾限的上限對齊 chassis.lift_top_deg（開迴路那組已經在用的上限）。
// 在 initialize() 裡叫一次就好——不能寫成建構式的初值，因為 chassis 是另一個
// 編譯單元的全域物件，全域初始化順序沒有保證。
void lift_pid_sync_limits();
