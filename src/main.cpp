#include "main.h"
#include "vexdash_pros/vexdash_pros.h" // vexdash car-side telemetry library
#include "tune.h"                      // JAR-style tune mode (Brain TUNE button + dashboard tunables)

void initialize() {
	// Load real PID constants first so the dashboard sliders start at the
	// values your code actually uses (not 0).
	default_constants();

	// The rotation sensor keeps its reading across program runs; make the
	// arm's starting (fully down) position 0 deg.
	arm_rotation.reset_position();

	// --- vexdash: JAR-style tune mode ---------------------------------------
	// 一行登記完整套可調參數（drive / heading / turn / swing 四組 PID ＋各自的
	// 進階退出條件 ＋ controller 測試值）與四顆測試按鈕。名字與分組逐字照
	// D:\VEX v5\AI專案\DPLIB\jar-template\src\JAR-Template\tune.cpp，所以網頁上
	// 的分組跟範例車長得一樣。清單與「JAR 有、這台車沒有」的略過項見 src/tune.cpp。
	// ⚠ 一定要在 init_smartport() 之「前」：vexdash 的規矩是先登記再啟動。
	tune_register_dashboard();

	// --- vexdash: live graph channels (streamed to the web Graph panel) ---
	vexdash::watch("drive_error",  &chassis.drive_error,       "in");
	vexdash::watch("drive_target", &chassis.tele_drive_target, "in");
	vexdash::watch("drive_output", &chassis.tele_drive_output, "V");
	vexdash::watch("turn_error",   &chassis.tele_turn_error,   "deg");
	vexdash::watch("turn_target",  &chassis.tele_turn_target,  "deg");
	vexdash::watch("turn_output",  &chassis.tele_turn_output,  "V");

	// --- vexdash: stream lift1 onto the Graph: pos/rpm/temp/amp ---
	vexdash::watch_motor("lift1", lift1);

	// --- vexdash: Device Map -- sensors section ---
	vexdash::declare_device(arm_rotation.get_port(), vexdash::DeviceType::kRotation, "arm_rotation");

	// Start vexdash over the ESP32 Smart Port bridge (port 11 @ 921600 baud).
	// The ESP32 relays telemetry to the dashboard over WiFi (ws://192.168.4.1).
	// Port 11 is free — nothing in robot-config.cpp claims it.
	// Smart Port path leaves stdout free (printf still works); HUD off by default.
	// Baud MUST match the WiFi box firmware's Serial1.begin(921600, ...).
	// Running 115200 here starves the link and the dashboard drops every ~5 s.
	vexdash::init_smartport(11, 921600);

	start_dashboard();

	// 調參模式的兩個背景小任務（一個負責開車跑測試、一個負責讀控制器按鍵）。
	// 一定要在 init_smartport() 之「後」：登記完才起跑。
	// 模式本身預設關著，沒按 Brain 螢幕上的 TUNE 鈕前這兩個任務什麼都不做。
	tune_start();
}

// inertial.tare_euler() and inertial.tare() both work here. Neither takes
// effect if called from competition_initialize() or disabled() before the IMU
// has finished calibrating, hence the delays.
void disabled() {
	delay(1000);
	inertial.tare_euler();
}

void competition_initialize() {
	delay(2500); // wait for imu to calibrate
	// chassis_lemlib.calibrate() is deliberately NOT called: it changes the
	// units DriveL/DriveR.get_position() report, which breaks Drive's odometry.
	delay(2250);
	inertial.tare_euler();
}

void autonomous() {
	chassis.set_coordinates(0, 0, 0);

	//route: whichever is selected on the dashboard's Auton Select tab
	switch (selected_auton) {
		case AutonRoutine::left:
			left();
			break;
		case AutonRoutine::left2:
			left2();
			break;
		case AutonRoutine::right:
			right();
			break;
		case AutonRoutine::right2:
			right2();
			break;
		case AutonRoutine::skill:
			skill();
			break;
	}
}

void opcontrol() {
	// ⚠ 這裡**不可以**再叫 default_constants()（2026-09-13 拿掉）。
	// 它會把 robot-config.cpp 裡硬寫的那組 PID 蓋回去，也就是說在 dashboard
	// 上調好的每一個值，只要比賽切換一次模式（disabled -> driver）或程式重跑，
	// 就全部被還原成舊值——教練會看到「明明調過了，車子跑起來還是原來那樣」。
	// 常數現在只在 initialize() 載入一次（本檔第 8 行），之後就只有 dashboard
	// 會動它。定案的數字還是要自己抄回 src/robot-config.cpp。
	tune_start();   // 保險：initialize() 已經叫過，這裡再叫一次也只會起一次

	// 兩種模式，由 Brain 螢幕 POSITION 分頁右上角那顆 TUNE 鈕切換：
	//   TUNE OFF（預設）= 原本的駕駛，chassis.control_arcade() 一個字都沒改。
	//   TUNE ON         = 調參模式，搖桿照樣能開車，但 L1/L2/R1/R2/X/Y/A/B 變成
	//                     「跑一次測試動作」的按鈕（配置表在 src/tune.cpp），
	//                     機構全程 hold 不吃按鍵。電腦 dashboard 上的四顆按鈕
	//                     （Run drive/turn/swing test、Run STOP）同時也有效。
	//
	// ⚠ 這兩個函式各自都是「跑到模式切換才返回」的無窮迴圈
	//   （control_arcade() 本來就是這樣寫的，它在 src/Template/drive.cpp），
	//   所以外面這一圈其實只在切換模式的那一瞬間跑到。
	//
	// ⚠ 在 dashboard 上調好的 kP/kI/kD 只活在記憶體裡：關機就沒了。
	//   定案之後要把數字抄回 src/robot-config.cpp 的 default_constants()。
	while (true) {
		if (tune_mode_enabled()) tune_drive_loop();
		else                     chassis.control_arcade();
		delay(10);
	}
}