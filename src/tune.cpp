#include "main.h"
#include "tune.h"
#include "lift_pid.h"
#include "vexdash_pros/vexdash_pros.h"

#include <cmath>
#include <cstdint>

/*===========================================================================*/
/*  JAR Template 的調參模式，搬到 PROS。                                      */
/*                                                                           */
/*  原版：D:\VEX v5\AI專案\DPLIB\jar-template\src\JAR-Template\tune.cpp       */
/*  這一份刻意跟原版長得很像（同樣的 job 編號、同樣的 10 秒上限、同樣的參數   */
/*  名字與分組），差異只有三處，每一處下面都寫了理由：                        */
/*    1. JAR 的機構叫 D4B、這台車的機構是 lift（手臂），所以 JAR 的 d4b 四組   */
/*       參數與「Run D4B test」按鈕改名成 lift（見 include/lift_pid.h）。      */
/*       2026-09-13 補：原本這裡寫「這台車沒有機構位置 PID 所以整組略過」，    */
/*       教練要調 lift 的 PID，所以照 JAR 的 Mech 移植了一份過來。            */
/*    2. *_max_voltage 的範圍是 0~127 不是 0~12：這台車的 Drive 輸出走的是     */
/*       pros::Motor::move()（-127~127），default_constants() 填的就是         */
/*       127 / 64 / 107。照 JAR 夾成 12 會直接把底盤廢掉。                     */
/*    3. dashboard 上那幾顆按鈕不帶參數：PROS 版 vexdash 的一行式              */
/*       declare_command() 只支援「零參數觸發鈕」（見 vexdash_pros.h），       */
/*       所以「要走幾吋 / 轉到幾度」改由 "controller" 那一組滑桿決定           */
/*       ——那組滑桿本來就是 JAR 拿來餵控制器按鍵的同一批值。                  */
/*===========================================================================*/

namespace {

// ---- 測試動作的編號（照抄 JAR，方便兩邊對讀）----
const int kJobNone   = 0;
const int kJobDrive  = 1;
const int kJobTurn   = 2;
const int kJobSwing  = 3;  // 左 swing
const int kJobLift   = 4;  // 手臂位置 PID（JAR 的 4 就是它的 D4B，編號刻意對齊）
const int kJobSwingR = 5;  // 右 swing（JAR 也是 5）

// 一次測試動作最多跑多久。時間到自動停——這是「車子一定停得下來」的最後一道保險。
const std::uint32_t kMaxRunMs = 10000;

// 按下測試鈕之後、真的開跑之前要等多久。
// 這一段是為了讓起跑條件跟 auton 一樣「從靜止開始」，理由寫在 tune_job_task()
// 裡真正用到它的地方。brake 模式下這台車放掉搖桿約 100ms 內就停住，150 有餘裕。
const std::uint32_t kSettleBeforeRunMs = 150;

// 下面這些會被三個執行緒同時碰（vexdash 背景緒、tune_job_task、opcontrol），
// 所以要 volatile：叫編譯器每次老實去記憶體讀，不要把值留在暫存器。
volatile int           g_job          = kJobNone;
volatile float         g_job_value    = 0;
volatile bool          g_stop_request = false;
volatile std::uint32_t g_job_start_ms = 0;
volatile bool          g_task_alive   = false;
volatile bool          g_ctl_mode     = false;

// 控制器每一顆鍵要跑多遠／轉到幾度。不寫死：場地大小與要測的範圍每次都不一樣，
// 這幾個在電腦端 dashboard 的 "controller" 那一組就可以即時改。
float g_ctl_drive_inch = 24;                      // L1 前進 / L2 後退 幾吋
float g_ctl_swing_deg  = 90;                      // R1 左 / R2 右 swing 幾度（相對）
float g_turn_deg[4]    = { 0, 45, 90, 180 };      // X / Y / A / B 的絕對方位

// 手臂的四個高度（方向鍵四顆各一個），照機構真實高低排：
//   [3] 上鍵 = 最高    [2] 右鍵    [1] 左鍵    [0] 下鍵 = 最低
// 電腦上的名字是 lift_deg_1 ~ lift_deg_4，數字由低到高（1 最低、4 最高）。
// 單位＝控制器螢幕上那個 "arm: xxx"（見 include/lift_pid.h 的口徑說明）。
//
// 2026-09-13：預設 0 / 90 / 180 / 270 是照 lift_top_deg（270）四等分猜的，
//   **不是**量過的實際高度。上車之後用手把手臂擺到你要的位置、看控制器螢幕
//   的 arm: 數字抄下來，再回電腦上把這四格改掉。
//   真正的上下限還會被 lift_pid.min_position / max_position 夾一次（雙保險）。
float g_lift_deg[4]    = { 0, 90, 180, 270 };     // 下, 左, 右, 上

/**
 * 一個可以在電腦上拉滑桿調整的常數。
 * name  = 網頁上顯示的名字（跟 JAR 逐字相同）
 * group = 分在哪一組（跟 JAR 逐字相同，分組長相才會一樣）
 * value = 指向 chassis 裡那個常數，滑桿一動就直接寫進去
 * min / max = 允許範圍。PROS 版 watch_config 沒有夾限功能，所以由下面
 *             tune_job_task 每 20ms 夾一次（打錯一個 0 不會變成意外）。
 */
struct TuneParam {
  const char* name;
  const char* group;
  float*      value;
  float       min;
  float       max;
};

TuneParam kParams[] = {
    // ---- 常用（名字＝include/Template/drive.h 裡的變數名，調好直接抄回 robot-config.cpp）----
    {"drive_kp",          "drive", &chassis.drive_kp,          0,  50},
    {"drive_ki",          "drive", &chassis.drive_ki,          0,   5},
    {"drive_kd",          "drive", &chassis.drive_kd,          0, 100},
    {"drive_starti",      "drive", &chassis.drive_starti,      0,  90},
    // ⚠ 0~127 不是 JAR 的 0~12，理由見檔頭差異 2。
    {"drive_max_voltage", "drive", &chassis.drive_max_voltage, 0, 127},

    {"heading_kp",          "heading", &chassis.heading_kp,          0,  50},
    {"heading_ki",          "heading", &chassis.heading_ki,          0,   5},
    {"heading_kd",          "heading", &chassis.heading_kd,          0, 100},
    {"heading_starti",      "heading", &chassis.heading_starti,      0,  90},
    {"heading_max_voltage", "heading", &chassis.heading_max_voltage, 0, 127},

    {"turn_kp",          "turn", &chassis.turn_kp,          0,  50},
    {"turn_ki",          "turn", &chassis.turn_ki,          0,   5},
    {"turn_kd",          "turn", &chassis.turn_kd,          0, 100},
    {"turn_starti",      "turn", &chassis.turn_starti,      0,  90},
    {"turn_max_voltage", "turn", &chassis.turn_max_voltage, 0, 127},

    {"swing_kp",          "swing", &chassis.swing_kp,          0,  50},
    {"swing_ki",          "swing", &chassis.swing_ki,          0,   5},
    {"swing_kd",          "swing", &chassis.swing_kd,          0, 100},
    {"swing_starti",      "swing", &chassis.swing_starti,      0,  90},
    {"swing_max_voltage", "swing", &chassis.swing_max_voltage, 0, 127},

    // ---- 進階：什麼時候算「到了」、最多跑多久 ----
    // timeout 下限 500ms：設成 0 等於「永遠不逾時」，車子就停不下來了。
    {"drive_settle_error", "drive 進階", &chassis.drive_settle_error,   0,    20},
    {"drive_settle_time",  "drive 進階", &chassis.drive_settle_time,    0,  2000},
    {"drive_timeout",      "drive 進階", &chassis.drive_timeout,      500, 15000},

    {"turn_settle_error", "turn 進階", &chassis.turn_settle_error,   0,    20},
    {"turn_settle_time",  "turn 進階", &chassis.turn_settle_time,    0,  2000},
    {"turn_timeout",      "turn 進階", &chassis.turn_timeout,      500, 15000},

    {"swing_settle_error", "swing 進階", &chassis.swing_settle_error,   0,    20},
    {"swing_settle_time",  "swing 進階", &chassis.swing_settle_time,    0,  2000},
    {"swing_timeout",      "swing 進階", &chassis.swing_timeout,      500, 15000},

    // ---- 控制器測試模式：每一顆鍵跑多少 ----
    // 不是 PID 常數，是「按一下 L1 要走幾吋」這種測試用的目標值。
    // ctl_drive_inch 上限 72 吋＝六呎，比半個場還長；ctl_swing_deg 上限 180
    //（swing 走最短路徑，超過 180 的相對角度沒有意義）；兩個下限都不給 0
    //（按了車子不動比按了亂動更難查）。
    {"ctl_drive_inch", "controller", &g_ctl_drive_inch, 1,  72},
    {"ctl_swing_deg",  "controller", &g_ctl_swing_deg,  1, 180},

    // 轉彎四顆鍵的角度。這是**絕對方位**，所以下限到 -180、上限 360。
    // turn_deg_1 預設 0 ＝「轉回起始方位」，最容易看出過衝與來回震盪，不要以為是沒設定。
    {"turn_deg_1", "controller", &g_turn_deg[0], -180, 360},
    {"turn_deg_2", "controller", &g_turn_deg[1], -180, 360},
    {"turn_deg_3", "controller", &g_turn_deg[2], -180, 360},
    {"turn_deg_4", "controller", &g_turn_deg[3], -180, 360},

    // 手臂四個高度（1 最低 ~ 4 最高），方向鍵四顆各一個。夾限給 -10~360 是
    // 為了留一點餘裕讓人試，真正會不會跑到那裡由 lift 安全那組決定：
    // LiftPID::move_to() 會再用 lift_min_pos / lift_max_pos 夾一次。
    {"lift_deg_1", "controller", &g_lift_deg[0], -10, 360},
    {"lift_deg_2", "controller", &g_lift_deg[1], -10, 360},
    {"lift_deg_3", "controller", &g_lift_deg[2], -10, 360},
    {"lift_deg_4", "controller", &g_lift_deg[3], -10, 360},

    // ---- lift 手動：教練原本在調的開迴路手臂控制 ----
    // （Drive::control_arcade()，src/Template/drive.cpp；平常駕駛走的就是這組）
    // 常數本體是 chassis 的成員（include/Template/drive.h），這裡只是照抄上面
    // 幾組的寫法拿位址註冊；預設值＝原本寫死在 control_arcade() 裡的值。
    //
    // ⚠ 2026-09-13 分組名從 "lift" 改成 "lift 手動"：下面新增的位置 PID 那組
    //   照 JAR 的規矩就是叫 "lift"，兩組同名會在網頁上混成一堆分不出誰是誰。
    //   **參數名字一個都沒改**，所以抄回 robot-config.cpp 的對照關係不變。
    {"lift_hold_ff",        "lift 手動", &chassis.lift_hold_ff,        0,  60},
    {"lift_slew",           "lift 手動", &chassis.lift_slew,           1,  60},
    {"lift_up_volt",        "lift 手動", &chassis.lift_up_volt,        0, 127},
    {"lift_down_volt",      "lift 手動", &chassis.lift_down_volt,   -127,   0},
    {"lift_down_volt_claw", "lift 手動", &chassis.lift_down_volt_claw, -127, 0},
    {"lift_top_deg",        "lift 手動", &chassis.lift_top_deg,        0, 360},
    {"lift_bottom_deg",     "lift 手動", &chassis.lift_bottom_deg,     0,  60},
    // 原本是 uint32_t，watch_config 不支援 uint32_t*（見
    // include/vexdash_pros/watch_registry.h 的 add_config 多載），已在
    // include/Template/drive.h 把型別改成 float，這裡直接註冊同一個成員。
    {"lift_double_tap_ms",  "lift 手動", &chassis.lift_double_tap_ms, 100, 2000},

    // ---- lift：手臂的位置 PID（JAR 的 d4b 那四組，字首改 lift）----
    // 本體在 include/lift_pid.h / src/lift_pid.cpp。只在 tune 模式生效，
    // 平常駕駛的 L1/L2 走的還是上面那組 "lift 手動"，一個字都沒動。
    //
    // 誤差單位是「手臂度數」（全行程約 0~270），不是 JAR 的馬達編碼器度數
    // （動輒好幾千），所以 kp 是個位數而不是 0.05 那種小數。
    // ⚠ lift_max_voltage 的範圍是 0~127 不是 JAR 的 0~12：這台車的輸出走
    //   pros::Motor::move()，理由同檔頭差異 2。
    {"lift_kp",          "lift", &lift_pid.kp,          0,  20},
    {"lift_ki",          "lift", &lift_pid.ki,          0,   2},
    {"lift_kd",          "lift", &lift_pid.kd,          0,  50},
    {"lift_starti",      "lift", &lift_pid.starti,      0,  90},
    {"lift_max_voltage", "lift", &lift_pid.max_voltage, 0, 127},

    // ---- 抗重力前饋：輸出 = kg * cos(手臂跟水平差幾度) ----
    // lift_kg 預設 12 ＝ 現行的 lift_hold_ff（已知「手放開不會掉」的最小輸出）。
    // lift_deg_per_motor_deg 是 0 的話角度永遠算成 0、cos(0)=1，整條退化成
    // 「固定給 lift_kg」——就是現在開迴路在做的事，也是建議的起點。
    // 感測器直接裝在手臂軸上的話這一格填 1，才會依角度變化（要實車量）。
    {"lift_kg",                "lift 抗重力", &lift_pid.kg,                0,  60},
    {"lift_horizontal_deg",    "lift 抗重力", &lift_pid.horizontal_deg, -360, 360},
    {"lift_deg_per_motor_deg", "lift 抗重力", &lift_pid.deg_per_motor_deg, 0,   2},

    // ---- 進階：什麼時候算「到了」、最多跑多久 ----
    // lift_timeout 下限一樣是 500ms（0 = 永不逾時，手臂會停不下來）。
    // lift_hold_ms 是到位之後再撐住幾毫秒——這一段就是你在曲線上看「會不會掉」
    // 的地方，也是調 lift_kg 的依據。
    {"lift_settle_error", "lift 進階", &lift_pid.settle_error,   0,   30},
    {"lift_settle_time",  "lift 進階", &lift_pid.settle_time,    0, 2000},
    {"lift_timeout",      "lift 進階", &lift_pid.timeout,      500, 8000},
    {"lift_hold_ms",      "lift 進階", &lift_pid.hold_ms,        0, 3000},

    // ---- 安全：只准跑到這兩個角度之間 ----
    // 打錯一個 0（想打 27 打成 270）不會讓手臂去撞機械死點硬卡住。
    // 上限預設由 lift_pid_sync_limits() 對齊 lift_top_deg（見 src/lift_pid.cpp）；
    // 那個值要在真車上量過才准往上調。
    {"lift_min_pos",     "lift 安全", &lift_pid.min_position, -10, 360},
    {"lift_max_pos",     "lift 安全", &lift_pid.max_position, -10, 360},
};
const int kParamCount = sizeof(kParams) / sizeof(kParams[0]);

// PROS 版 watch_config 沒有 min/max（JAR 的 VEXcode 版是在 callback 裡夾）。
// 所以改成由背景小任務每 20ms 掃一次表，超出範圍就夾回來。
void clamp_params() {
  for (int i = 0; i < kParamCount; i++) {
    float v = *kParams[i].value;
    if (v < kParams[i].min)      *kParams[i].value = kParams[i].min;
    else if (v > kParams[i].max) *kParams[i].value = kParams[i].max;
  }
}

/**
 * 真正把「要跑什麼」記下來的地方。電腦上的按鈕與控制器上的按鍵都走這一個函式，
 * 所以兩邊排的是同一個工作、吃同一個 10 秒上限、被同一顆 STOP 停掉。
 * 這裡刻意不開車：開車一律由下面的 tune_job_task 做（這個函式可能跑在
 * vexdash 的背景執行緒上，卡住它會讓連線斷掉）。
 */
void begin_job(int job, float value) {
  if (g_job != kJobNone) return;  // 上一次還沒跑完，先不要疊上去
  g_job_value    = value;
  g_stop_request = false;
  g_job_start_ms = pros::millis();
  g_job          = job;
}

// ---- 電腦 dashboard 上那四顆按鈕（名字逐字照 JAR）----
// 走多遠／轉到哪，用的是 "controller" 那一組滑桿的當下值（理由見檔頭差異 3）。
void on_run_drive(void*) { begin_job(kJobDrive, g_ctl_drive_inch); }
void on_run_turn(void*)  { begin_job(kJobTurn,  g_turn_deg[0]); }
void on_run_swing(void*) {
  // swing 的角度是**相對現在的方位**，跟 turn 不一樣（JAR 的 R1 也是這樣算）。
  begin_job(kJobSwing, chassis.get_absolute_heading() - g_ctl_swing_deg);
}
// 手臂跑到 lift_deg_1（"controller" 那組的第一格，＝方向鍵下鍵那個位置）。
// JAR 的「Run D4B test」是按鈕帶參數的，PROS 版的 declare_command() 只支援
// 零參數觸發鈕（檔頭差異 3），所以目標值一樣是從 "controller" 那組讀當下值。
void on_run_lift(void*) { begin_job(kJobLift, g_lift_deg[0]); }
void on_stop(void*) { g_stop_request = true; }

/**
 * 背景小任務：發現有人按了測試按鈕，就開車跑一次。
 * 跑的是 Drive 原本的 drive_distance / turn_to_angle / swing_to_angle，
 * 跟自動程式裡用的是同一套函式，所以調出來的數值直接可以用。
 */
void tune_job_task() {
  g_task_alive = true;
  while (true) {
    clamp_params();

    int job = g_job;
    if (job != kJobNone) {
      if (pros::competition::is_autonomous()) {
        // 自動階段不准跑：那時候 auton 自己也在寫馬達電壓，兩邊會打架。
        g_job = kJobNone;
      } else {
        float value = g_job_value;

        // ---- 起跑條件要跟 auton 一模一樣：從「真的靜止」開始 ----------------
        // 2026-09-13 教練實測：「在 dashboard 調 drive 時底盤的速度變化，跟沒用
        // dashboard 跑底盤明顯不一樣」。原因之一就在這裡：原本 drive_stop() 的
        // 下一行就直接開跑，可是搖桿在上一個 10ms tick 才剛餵過馬達
        // （tune_drive_loop()），車子還在滑。drive_distance() 一進函式就把
        // start_average_position 與 get_absolute_heading() 當成整段的基準
        // （src/Template/drive.cpp 的 306 / 291 行），基準抓到的是「移動中」的
        // 值 —— 距離會短一截、heading PID 一開始就有假誤差，整條加減速曲線就跟
        // auton 從靜止起跑的那條對不起來。
        // 修法：煞停之後等它真的停下來，再讓 drive_distance() 去量基準。
        chassis.drive_stop(MotorBrake::brake);
        pros::delay(kSettleBeforeRunMs);

        if (job == kJobLift) {
          // 手臂測試：底盤整段不動（上面已經 brake 住），只有 lift1/lift2 在跑。
          // move_to() 會卡到到位／逾時／STOP／10 秒上限才回來——卡在這個背景
          // 小任務裡是對的，dashboard 的 callback 與控制器任務都不會被拖住。
          lift_pid.move_to(value);
          lift_pid.stop();
        }
        // ⚠ 一律走「跟 auton 完全同一個函式、同一個 overload」。
        //   auton 寫的是帶 heading 的 drive_distance(距離, 方位, motion_chaining)，
        //   所以這裡也用它，heading = 車現在的方位（上面已經停穩才量）。
        //   單參數的 drive_distance(value) 最後也是轉呼叫這一個
        //   （src/Template/drive.cpp:290-292），寫成明的是為了以後有人動了單參數
        //   版時，調參跟 auton 不會悄悄分岔成兩條路。
        //   turn / swing 同理，motion_chaining 一律明寫 false（＝auton 的預設）。
        else if (job == kJobDrive)  chassis.drive_distance(value, chassis.get_absolute_heading(), false);
        else if (job == kJobTurn)   chassis.turn_to_angle(value, false);
        else if (job == kJobSwing)  chassis.swing_to_angle(value, true,  false);
        else if (job == kJobSwingR) chassis.swing_to_angle(value, false, false);

        chassis.drive_stop(MotorBrake::brake);
        g_stop_request = false;
        g_job          = kJobNone;  // 跑完了，等下一次
      }
    }
    pros::delay(20);
  }
}

/*===========================================================================*/
/*  控制器測試模式：按鍵配置                                                  */
/*                                                                           */
/*  原則跟 JAR 一樣：**鍵在哪個方向，車子就往那個方向動**，學生不用背。       */
/*    L1  前進 ctl_drive_inch 吋      L2  後退 ctl_drive_inch 吋              */
/*    R1  往左 swing ctl_swing_deg    R2  往右 swing ctl_swing_deg            */
/*    X   轉到 turn_deg_1（預設 0）   Y   轉到 turn_deg_2（預設 45）           */
/*    A   轉到 turn_deg_3（預設 90）  B   轉到 turn_deg_4（預設 180）          */
/*                                                                           */
/*  方向鍵四顆 = 手臂的四個高度，照機構真實高低排（逐字照 JAR 的 D4B）：       */
/*    方向鍵上  手臂到 lift_deg_4（最高，預設 270）                            */
/*    方向鍵右  手臂到 lift_deg_3（預設 180）                                  */
/*    方向鍵左  手臂到 lift_deg_2（預設 90）                                   */
/*    方向鍵下  手臂到 lift_deg_1（最低，預設 0）                              */
/*  往上的鍵＝手臂往上，學生不用背。四個值自己量了在電腦上填。                */
/*  ⚠ 方向鍵右在平常駕駛是 toggle（control_arcade()），這裡不衝突：            */
/*    這個任務只在 g_ctl_mode 開著時才派工，而那時候跑的是 tune_drive_loop()， */
/*    control_arcade() 已經 break 掉了。                                       */
/*                                                                           */
/*  ⚠ 控制器上沒有 STOP 鍵（跟 JAR 一樣）。要停車有三條路：                   */
/*    1. 電腦 dashboard 上的 "Run STOP"                                       */
/*    2. 一次動作最多 kMaxRunMs = 10 秒，時間到自動停                          */
/*    3. Brain 電源鍵 / 拔電池（最終手段）                                     */
/*                                                                           */
/*  這個任務一定要跟 tune_job_task 分開：那個任務在開車時是卡在動作迴圈裡的   */
/*  （drive_distance 會一路跑到結束才回來），按鍵擠在同一個任務裡就讀不到。   */
/*===========================================================================*/
void tune_ctl_task() {
  pros::Controller ctl(pros::E_CONTROLLER_MASTER);

  // 上一圈每顆鍵是不是按著的。去抖（按住不放只算一次）就靠這一排。
  // 刻意不用 get_digital_new_press()：那個的「上一次」狀態是整台車共用的，
  // Drive::control_arcade() 也在用（L2 雙擊），兩邊會互相吃掉對方的邊緣。
  bool p_l1 = false, p_l2 = false, p_r1 = false, p_r2 = false;
  bool p_x = false, p_y = false, p_a = false, p_b = false;
  bool p_up = false, p_dn = false, p_lf = false, p_rt = false;

  while (true) {
    // ⚠ 不管在不在測試模式，每一圈都要照樣讀、照樣更新 p_*。
    //   否則「按著某顆鍵的時候才把模式打開」會被看成剛剛按下去而馬上開車。
    bool l1 = ctl.get_digital(DIGITAL_L1);
    bool l2 = ctl.get_digital(DIGITAL_L2);
    bool r1 = ctl.get_digital(DIGITAL_R1);
    bool r2 = ctl.get_digital(DIGITAL_R2);
    bool bx = ctl.get_digital(DIGITAL_X);
    bool by = ctl.get_digital(DIGITAL_Y);
    bool ba = ctl.get_digital(DIGITAL_A);
    bool bb = ctl.get_digital(DIGITAL_B);
    bool up = ctl.get_digital(DIGITAL_UP);
    bool dn = ctl.get_digital(DIGITAL_DOWN);
    bool lf = ctl.get_digital(DIGITAL_LEFT);
    bool rt = ctl.get_digital(DIGITAL_RIGHT);

    bool hit_l1 = l1 && !p_l1;
    bool hit_l2 = l2 && !p_l2;
    bool hit_r1 = r1 && !p_r1;
    bool hit_r2 = r2 && !p_r2;
    bool hit_x  = bx && !p_x;
    bool hit_y  = by && !p_y;
    bool hit_a  = ba && !p_a;
    bool hit_b  = bb && !p_b;
    bool hit_up = up && !p_up;
    bool hit_dn = dn && !p_dn;
    bool hit_lf = lf && !p_lf;
    bool hit_rt = rt && !p_rt;

    p_l1 = l1; p_l2 = l2; p_r1 = r1; p_r2 = r2;
    p_x  = bx; p_y  = by; p_a  = ba; p_b  = bb;
    p_up = up; p_dn = dn; p_lf = lf; p_rt = rt;

    // 接上正式場地控制器就自動關掉：比賽中誤觸測試動作是災難。
    // 練習用的 Competition Switch 不算（is_field_control 只認正式場控）。
    if (pros::competition::is_field_control()) g_ctl_mode = false;

    if (g_ctl_mode && !pros::competition::is_autonomous()) {
      float heading = chassis.get_absolute_heading();
      if      (hit_l1) begin_job(kJobDrive,   g_ctl_drive_inch);
      else if (hit_l2) begin_job(kJobDrive,  -g_ctl_drive_inch);
      // 往左＝方位減、動左邊那組輪子；往右＝方位加、動右邊那組。左右完全對稱。
      else if (hit_r1) begin_job(kJobSwing,   heading - g_ctl_swing_deg);
      else if (hit_r2) begin_job(kJobSwingR,  heading + g_ctl_swing_deg);
      else if (hit_x)  begin_job(kJobTurn,    g_turn_deg[0]);
      else if (hit_y)  begin_job(kJobTurn,    g_turn_deg[1]);
      else if (hit_a)  begin_job(kJobTurn,    g_turn_deg[2]);
      else if (hit_b)  begin_job(kJobTurn,    g_turn_deg[3]);
      // ---- 手臂：四個高度，照方向鍵的高低排 ----
      else if (hit_up) begin_job(kJobLift,    g_lift_deg[3]);   // 最高
      else if (hit_rt) begin_job(kJobLift,    g_lift_deg[2]);
      else if (hit_lf) begin_job(kJobLift,    g_lift_deg[1]);
      else if (hit_dn) begin_job(kJobLift,    g_lift_deg[0]);   // 最低
    }

    pros::delay(20);
  }
}

}  // namespace

/*===========================================================================*/
/*  對外的介面                                                                */
/*===========================================================================*/

void tune_register_dashboard() {
  for (int i = 0; i < kParamCount; i++) {
    vexdash::watch_config(kParams[i].name, kParams[i].value, kParams[i].group);
  }
  // 按鈕名字逐字照 JAR（src/JAR-Template/tune.cpp 的 register_all）。
  // JAR 的「Run D4B test」在這台車叫「Run lift test」（機構名字不一樣）。
  vexdash::declare_command("Run drive test", on_run_drive);
  vexdash::declare_command("Run turn test",  on_run_turn);
  vexdash::declare_command("Run swing test", on_run_swing);
  vexdash::declare_command("Run lift test",  on_run_lift);
  vexdash::declare_command("Run STOP",       on_stop);

  // 手臂安全夾限的上限對齊開迴路那組的 lift_top_deg。要在這裡叫（initialize()
  // 執行期）而不是寫成建構式初值：chassis 是另一個編譯單元的全域物件，
  // 全域初始化順序沒有保證。這一行也讓網頁上的 lift_max_pos 滑桿一開始就
  // 停在正確的位置，而不是標頭檔裡那個硬寫的 270。
  lift_pid_sync_limits();
}

void tune_start() {
  // function-local static：呼叫幾次都只會真的起一次。
  // ⚠ 優先權與堆疊一律用 PROS 預設（TASK_PRIORITY_DEFAULT / TASK_STACK_DEPTH_DEFAULT）
  //   ——opcontrol()、autonomous() 這兩個主執行緒用的就是同一組預設值。
  //   把 job_runner 調高會讓它插隊、調低會被搖桿迴圈壓住，兩種都會讓測試動作的
  //   迴圈週期跟 auton 對不起來（週期為什麼要緊，見 src/Template/drive.cpp 的
  //   drive_distance()）。要動之前先讀那段註解。
  static pros::Task job_runner(tune_job_task, TASK_PRIORITY_DEFAULT,
                               TASK_STACK_DEPTH_DEFAULT, "tune_job");
  static pros::Task ctl_runner(tune_ctl_task, TASK_PRIORITY_DEFAULT,
                               TASK_STACK_DEPTH_DEFAULT, "tune_ctl");
}

bool tune_mode_enabled() { return g_ctl_mode; }

void tune_mode_set(bool on) {
  // 關掉的時候順手按一次 STOP：不然會變成「按鍵已經不管用了，可是上一個
  // 按鍵叫出來的動作還在跑」，很難解釋。走的是同一面 g_stop_request 旗子。
  if (!on) g_stop_request = true;
  g_ctl_mode = on;
}

bool tune_task_started() { return g_task_alive; }

bool tune_stop_requested() {
  // 不是測試動作（學生自己的 auton）就不要管；小任務沒起來時 g_job 沒人清，
  // 也一律不管，免得學生的 auton 每個動作都被誤停。
  if (g_job == kJobNone || !g_task_alive) return false;
  if (g_stop_request) return true;
  if (pros::millis() - g_job_start_ms > kMaxRunMs) {
    g_stop_request = true;  // 跑太久了，強制停
    return true;
  }
  return false;
}

bool tune_is_running() {
  if (g_job == kJobNone) return false;
  if (!g_task_alive) { g_job = kJobNone; return false; }
  if (pros::millis() - g_job_start_ms > kMaxRunMs + 500) {
    // 到這裡表示 10 秒的停車要求沒生效。再催一次，然後把搖桿還給駕駛。
    // ⚠ 這裡「不可以」把 g_job 清成 0：清了的話 tune_stop_requested() 第一句
    //   就會回 false，動作迴圈反而永遠不會 break，變成迴圈跟搖桿同時寫馬達。
    g_stop_request = true;
    return false;
  }
  return true;
}

void tune_drive_loop() {
  pros::Controller ctl(pros::E_CONTROLLER_MASTER);

  // 機構不吃搖桿按鍵：那幾顆鍵在調參模式下已經是測試按鈕了。
  // 手臂是唯一的例外——方向鍵會叫它跑位置 PID 測試，那段期間不可以 brake。
  lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  toggle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_roller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_roller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  // 底盤用 brake：兩次測試之間停得住，起跑點才量得準。
  chassis.drive_stop(MotorBrake::brake);

  while (tune_mode_enabled()) {
    // 正在跑測試動作時不要碰底盤與手臂，不然這個迴圈每 10ms 就把測試的輸出
    // 蓋掉一次——手臂那兩行尤其致命：LiftPID::move_to() 才剛 move() 完，
    // 這裡馬上 brake()，PID 會變成「輸出永遠是 0」，手臂一動也不動。
    if (tune_is_running()) {
      // 2026-09-13：測試動作在跑的時候，這個迴圈**一顆馬達都不要碰**。
      // 原本 toggle / 兩顆 roller 的 brake() 在 if 外面，所以就算測試正在跑也
      // 照樣每 10ms 送三筆指令出去；那三筆跟 drive_distance() 的 PID 迴圈
      // （兩組編碼器 + IMU 讀取、兩組馬達寫入）在同一條 smart port 上排隊，
      // 把 PID 的一輪拖長。PID::compute() 的 kD／積分／settle 計時全都是
      // 「一輪算一次、而且假設一輪剛好 10ms」（src/Template/PID.cpp:101/106/111），
      // 輪子被拖長，同一組 kP/kD 打出來的加減速就跟 auton 不一樣——教練說的
      // 「速度變化明顯不同」，這是第二個原因（第一個是起跑點，見 tune_job_task）。
      // 機構在上一輪 idle 時就已經 brake 住了（brake mode 是持續生效的狀態，
      // 不是每輪都要重下的指令），少送這幾筆不會讓它們鬆掉。
      pros::delay(10);
      continue;
    }

    double throttle = ctl.get_analog(ANALOG_LEFT_Y);
    double turn     = ctl.get_analog(ANALOG_RIGHT_X);
    if (std::fabs(throttle) < 5) throttle = 0;
    if (std::fabs(turn) < 5)     turn = 0;
    chassis.DriveL.move(throttle + turn);
    chassis.DriveR.move(throttle - turn);

    lift1.brake();
    lift2.brake();
    toggle.brake();
    left_roller.brake();
    right_roller.brake();

    pros::delay(10);
  }

  chassis.drive_stop(MotorBrake::brake);
}

TuneCtlValues tune_ctl_values() {
  TuneCtlValues v;
  v.drive_inch = g_ctl_drive_inch;
  v.swing_deg  = g_ctl_swing_deg;
  for (int i = 0; i < 4; i++) v.turn_deg[i] = g_turn_deg[i];
  for (int i = 0; i < 4; i++) v.lift_deg[i] = g_lift_deg[i];
  return v;
}
