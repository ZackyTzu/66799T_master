#include "main.h"
#include "vexdash_pros/vexdash_pros.h" // vexdash car-side telemetry library

// vexdash PID-test controls: set target on the dashboard, flip the toggle to run.
double test_distance = 24.0;   // inches  — target for the drive PID test
double test_angle    = 90.0;   // degrees — target for the turn PID test
bool   run_drive_test = false; // toggle ON in dashboard to drive test_distance
bool   run_turn_test  = false; // toggle ON in dashboard to turn to test_angle

void initialize() {
	// Load real PID constants first so the dashboard sliders start at the
	// values your code actually uses (not 0).
	default_constants();

	// The rotation sensor keeps its reading across program runs; make the
	// arm's starting (fully down) position 0 deg.
	arm_rotation.reset_position();

	// --- vexdash: tunable PID gains (sliders on the web Config panel, auto write-back) ---
	vexdash::watch_config("kP", &chassis.drive_kp, "drive/pid");
	vexdash::watch_config("kI", &chassis.drive_ki, "drive/pid");
	vexdash::watch_config("kD", &chassis.drive_kd, "drive/pid");
	vexdash::watch_config("kP", &chassis.turn_kp, "turn/pid");
	vexdash::watch_config("kI", &chassis.turn_ki, "turn/pid");
	vexdash::watch_config("kD", &chassis.turn_kd, "turn/pid");

	// --- vexdash: live graph channels (streamed to the web Graph panel) ---
	vexdash::watch("drive_error",  &chassis.drive_error,       "in");
	vexdash::watch("drive_target", &chassis.tele_drive_target, "in");
	vexdash::watch("drive_output", &chassis.tele_drive_output, "V");
	vexdash::watch("turn_error",   &chassis.tele_turn_error,   "deg");
	vexdash::watch("turn_target",  &chassis.tele_turn_target,  "deg");
	vexdash::watch("turn_output",  &chassis.tele_turn_output,  "V");

	// --- vexdash: on-demand PID tests (set the target, toggle "run", watch the Graph) ---
	vexdash::watch_config("test_distance", &test_distance,  "drive/test"); // inches
	vexdash::watch_config("run_drive",     &run_drive_test, "drive/test"); // toggle ON to drive
	vexdash::watch_config("test_angle",    &test_angle,     "turn/test");  // degrees
	vexdash::watch_config("run_turn",      &run_turn_test,  "turn/test");  // toggle ON to turn

	// --- vexdash: stream lift1 onto the Graph: pos/rpm/temp/amp ---
	vexdash::watch_motor("lift1", lift1);

	// --- vexdash: Device Map -- sensors section ---
	vexdash::declare_device(arm_rotation.get_port(), vexdash::DeviceType::kRotation, "arm_rotation");

	// Start vexdash over the ESP32 Smart Port bridge (port 11 @ 115200 baud).
	// The ESP32 relays telemetry to the dashboard over WiFi (ws://192.168.4.1).
	// Port 11 is free — nothing in robot-config.cpp claims it.
	// Smart Port path leaves stdout free (printf still works); HUD off by default.
	vexdash::init_smartport(11, 115200);

	start_dashboard();
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
	default_constants();
	// The dashboard PID tests (run_drive_test / run_turn_test) are disabled.
	// To bring them back, replace the loop body with:
	//   static bool prev_drive = false, prev_turn = false;
	//   if (run_drive_test && !prev_drive)      chassis.drive_distance(test_distance);
	//   else if (run_turn_test && !prev_turn)   chassis.turn_to_angle(test_angle);
	//   else if (!run_drive_test && !run_turn_test) chassis.control_arcade();
	//   prev_drive = run_drive_test; prev_turn = run_turn_test;
	// A rising edge on a toggle then runs ONE test move; while a toggle stays
	// ON, arcade driving is paused.
	while (true) {
		chassis.control_arcade();
		delay(10);
	}
}